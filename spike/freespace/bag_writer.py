"""Bounded separate-process rosbag writer; storage cannot block ROS health callbacks."""

from collections import Counter
import json
import multiprocessing
from pathlib import Path
from queue import Empty, Full
import time


def writer_process(output: str, topics: dict, incoming, results) -> None:
    writer = None
    try:
        import rosbag2_py
        root = Path(output)
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(root / "bag"), storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
        for topic_id, (name, kind) in enumerate(topics.items()):
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=topic_id, name=name, type=kind, serialization_format="cdr"))
        counts, maximum_write_s = Counter(), 0.
        with (root / "receive_progress.jsonl").open("w", encoding="utf-8") as events:
            results.put({"ready": True})
            while True:
                item = incoming.get()
                if item is None:
                    break
                topic, payload, event = item
                begin = time.monotonic()
                writer.write(topic, payload, event["received_sim_ns"])
                maximum_write_s = max(maximum_write_s, time.monotonic()-begin)
                counts[topic] += 1
                events.write(json.dumps(event)+"\n")
        writer = None
        results.put({"complete": True, "topic_counts": dict(counts),
                     "maximum_storage_write_wall_s": maximum_write_s})
    except Exception as error:
        writer = None
        results.put({"complete": False, "failure": f"{type(error).__name__}: {error}"})


class BagWriter:
    def __init__(self, output: Path, topics: dict, policy: dict) -> None:
        context = multiprocessing.get_context("spawn")
        self.incoming = context.Queue(maxsize=policy["writer_queue_items"])
        self.results = context.Queue(maxsize=2)
        self.timeout = policy["writer_shutdown_wall_s"]
        self.process = context.Process(target=writer_process,
                                       args=(str(output), topics, self.incoming, self.results), daemon=True)
        self.process.start()
        try:
            ready = self.results.get(timeout=self.timeout)
            if not ready.get("ready"):
                raise ValueError(f"BAG_WRITER_START_FAILED:{ready}")
        except (Empty, ValueError):
            self.process.terminate()
            self.process.join(timeout=self.timeout)
            raise ValueError("BAG_WRITER_START_FAILED")

    def write(self, topic: str, payload: bytes, event: dict) -> None:
        if not self.process.is_alive():
            raise ValueError("BAG_WRITER_EXITED")
        try:
            self.incoming.put_nowait((topic, payload, event))
        except Full:
            raise ValueError("BAG_WRITER_BACKPRESSURE") from None

    def close(self) -> dict:
        try:
            self.incoming.put(None, timeout=self.timeout)
            self.process.join(timeout=self.timeout)
            if self.process.is_alive():
                raise ValueError("BAG_WRITER_SHUTDOWN_TIMEOUT")
            result = self.results.get(timeout=1.)
            if self.process.exitcode != 0 or not result.get("complete"):
                raise ValueError(f"BAG_WRITER_FAILED:{result}")
            return result
        except (Empty, Full):
            raise ValueError("BAG_WRITER_RESULT_MISSING") from None
        finally:
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=self.timeout)
            self.incoming.cancel_join_thread()
            self.incoming.close()
            self.results.close()
