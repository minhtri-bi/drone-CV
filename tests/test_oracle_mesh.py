from pathlib import Path
import struct

import numpy as np
import pytest

from drone_cv.oracle_mesh import mesh_vertices


SOURCE = """<source id="positions">
  <float_array id="array" count="15">999 888 1 2 3 777 666 4 5 6 555 444 7 8 9</float_array>
  <technique_common><accessor source="#array" count="3" offset="2" stride="5">
    <param name="X" type="float"/><param name="Y" type="float"/>
    <param name="Z" type="float"/>
  </accessor></technique_common>
</source>"""
GEOMETRY = f"""<geometry id="mesh"><mesh>{SOURCE}
  <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
  <triangles count="1"><input semantic="VERTEX" source="#vertices" offset="0"/>
    <p>0 1 2</p></triangles>
</mesh></geometry>"""
INSTANCE = '<instance_geometry url="#mesh" name="LandingFoam"/>'


def dae(tmp_path: Path, *, nodes: str = INSTANCE, geometry: str = GEOMETRY,
        asset: str = '<unit meter="1"/><up_axis>Z_UP</up_axis>',
        libraries: str = "", scene: str = '<instance_visual_scene url="#chosen"/>') -> Path:
    path = tmp_path / "fixture.dae"
    path.write_text(f"""<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
      <asset>{asset}</asset><library_geometries>{geometry}</library_geometries>
      {libraries}<library_visual_scenes><visual_scene id="chosen">
      <node id="root" name="Root">{nodes}</node></visual_scene></library_visual_scenes>
      <scene>{scene}</scene></COLLADA>""", encoding="utf-8")
    return path


def test_accessor_offset_stride_and_read_only_new_arrays(tmp_path):
    path = dae(tmp_path)
    first = mesh_vertices(path)
    second = mesh_vertices(path)
    assert len(first) == 1
    assert "LandingFoam" in first[0][0]
    np.testing.assert_array_equal(first[0][1], [[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    assert first[0][1].dtype == np.float64
    assert not first[0][1].flags.writeable
    assert not np.shares_memory(first[0][1], second[0][1])
    with pytest.raises(ValueError):
        first[0][1][0, 0] = 0


def test_unit_y_up_and_nested_order_have_independent_answer(tmp_path):
    nodes = f"""<translate>10 20 30</translate><rotate>0 0 1 90</rotate>
      <scale>2 3 4</scale><node id="child"><translate>1 0 0</translate>{INSTANCE}</node>"""
    points = mesh_vertices(dae(tmp_path, nodes=nodes,
                               asset='<unit meter="0.01"/><up_axis>Y_UP</up_axis>'))[0][1]
    # Child translation, parent scale, rotation, translation, then unit/basis conversion.
    np.testing.assert_allclose(points, [[0.04, -0.42, 0.24], [-0.05, -0.54, 0.30],
                                       [-0.14, -0.66, 0.36]], atol=1e-14)


def test_serialized_matrix_rows_column_vectors_and_scene_selection(tmp_path):
    nodes = f"""<matrix>0 -1 0 10 1 0 0 20 0 0 1 30 0 0 0 1</matrix>
      {INSTANCE}<node id="rubber"><translate>1 0 0</translate>
      <instance_geometry url="#mesh" name="LandingRubber"/></node>"""
    unused = '<geometry id="unused"><convex_mesh/></geometry>'
    other_scene = """<library_visual_scenes><visual_scene id="not-chosen">
      <node id="ignored"><instance_geometry url="#unused"/></node>
      </visual_scene></library_visual_scenes>"""
    instances = mesh_vertices(dae(tmp_path, nodes=nodes, geometry=GEOMETRY + unused,
                                  libraries=other_scene))
    assert len(instances) == 2
    assert len({name for name, _ in instances}) == 2
    assert "LandingFoam" in instances[0][0]
    assert "LandingRubber" in instances[1][0]
    np.testing.assert_array_equal(instances[0][1], [[8, 21, 33], [5, 24, 36], [2, 27, 39]])
    np.testing.assert_array_equal(instances[1][1], [[8, 22, 33], [5, 25, 36], [2, 28, 39]])


def test_lines_retain_positions_outside_triangle_and_ignore_unused_points(tmp_path):
    geometry = GEOMETRY.replace('count="15"', 'count="20"').replace(
        '7 8 9</float_array>', '7 8 9 0 0 100 200 300</float_array>').replace(
        'count="3" offset="2"', 'count="4" offset="2"').replace(
        '<p>0 1 2</p>', '<p>0 1 0</p>').replace(
        '</mesh>', '<lines count="1"><input semantic="VERTEX" source="#vertices" '
        'offset="0"/><p>1 2</p></lines></mesh>')
    points = mesh_vertices(dae(tmp_path, geometry=geometry))[0][1]
    np.testing.assert_array_equal(points, [[1, 2, 3], [4, 5, 6], [7, 8, 9]])


def test_local_node_instances_keep_distinct_paths(tmp_path):
    libraries = f'<library_nodes><node id="shared">{INSTANCE}</node></library_nodes>'
    nodes = """<node id="left"><instance_node url="#shared"/></node>
      <node id="right"><translate>10 0 0</translate><instance_node url="#shared"/></node>"""
    instances = mesh_vertices(dae(tmp_path, nodes=nodes, libraries=libraries))
    assert len(instances) == 2
    assert instances[0][0] != instances[1][0]
    np.testing.assert_array_equal(instances[1][1] - instances[0][1], [[10, 0, 0]] * 3)


def test_accessor_named_components_are_not_assumed_to_be_xyz_order(tmp_path):
    geometry = GEOMETRY.replace('name="X"', 'name="Z"').replace('name="Z" type="float"/>\n  ',
                                                                          'name="X" type="float"/>\n  ')
    points = mesh_vertices(dae(tmp_path, geometry=geometry))[0][1]
    np.testing.assert_array_equal(points, [[3, 2, 1], [6, 5, 4], [9, 8, 7]])


def test_accessor_unnamed_parameters_and_interleaved_input_offsets(tmp_path):
    geometry = GEOMETRY.replace('offset="2" stride="5"', 'offset="0" stride="5"').replace(
        '<param name="X"', '<param type="float"/><param type="float"/><param name="X"')
    normal = """<source id="normals"><float_array id="normal-array" count="3">0 0 1</float_array>
      <technique_common><accessor source="#normal-array" count="1" stride="3">
      <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
      </accessor></technique_common></source>"""
    geometry = geometry.replace('<vertices', normal + '<vertices').replace(
        'source="#vertices" offset="0"/>',
        'source="#vertices" offset="1"/><input semantic="NORMAL" source="#normals" offset="0"/>').replace(
        '<p>0 1 2</p>', '<p>0 0 0 1 0 2</p>')
    points = mesh_vertices(dae(tmp_path, geometry=geometry))[0][1]
    np.testing.assert_array_equal(points, [[1, 2, 3], [4, 5, 6], [7, 8, 9]])


def test_nonphysical_camera_nodes_do_not_contribute_points(tmp_path):
    camera = '<library_cameras><camera id="camera"/></library_cameras>'
    nodes = INSTANCE + """<node id="camera-node">
      <matrix>1 0 0 100 0 1 0 200 0 0 1 300 0 0 0 1</matrix>
      <instance_camera url="#camera"/></node>"""
    instances = mesh_vertices(dae(tmp_path, nodes=nodes, libraries=camera))
    assert len(instances) == 1
    np.testing.assert_array_equal(instances[0][1], [[1, 2, 3], [4, 5, 6], [7, 8, 9]])


@pytest.mark.parametrize("change", [
    lambda text: text.replace('meter="1"', 'meter="0"'),
    lambda text: text.replace('Z_UP', 'X_UP'),
    lambda text: text.replace('<translate>1 2 3</translate>', '<lookat>0 0 0 0 0 1 0 1 0</lookat>'),
    lambda text: text.replace('<translate>1 2 3</translate>', '<skew>0 1 0 0 0 1 0</skew>'),
    lambda text: text.replace('<translate>1 2 3</translate>', '<rotate>0 0 0 90</rotate>'),
    lambda text: text.replace('<translate>1 2 3</translate>', '<translate>nan 0 0</translate>'),
    lambda text: text.replace('<translate>1 2 3</translate>', '<scale>1 2</scale>'),
    lambda text: text.replace('url="#mesh"', 'url="other.dae#mesh"'),
    lambda text: text.replace('source="#positions"', 'source="https://example.com/positions"'),
    lambda text: text.replace('url="#mesh"', 'url="#missing"'),
    lambda text: text.replace('<p>0 1 2</p>', '<p>0 1</p>'),
    lambda text: text.replace('<p>0 1 2</p>', '<p>0 1 3</p>'),
    lambda text: text.replace('<p>0 1 2</p>', '<p>0 1 -1</p>'),
    lambda text: text.replace('<p>0 1 2</p>', '<p>0 1 2.0</p>'),
    lambda text: text.replace('count="15"', 'count="14"'),
    lambda text: text.replace('count="3" offset="2"', 'count="4" offset="2"'),
    lambda text: text.replace('1 2 3 777', 'inf 2 3 777'),
    lambda text: text.replace('1 2 3 777', '1_0 2 3 777'),
    lambda text: text.replace('<node id="root"', '<node xml:base="other.dae" id="root"'),
    lambda text: text.replace('<translate>1 2 3</translate>', '<asset><unit meter="2"/></asset>'),
    lambda text: text.replace('<instance_geometry url="#mesh" name="LandingFoam"/>',
                              '<instance_geometry url="#mesh"><translate>1 0 0</translate></instance_geometry>'),
    lambda text: text.replace('<triangles', '<polylist').replace('</triangles>', '</polylist>'),
    lambda text: text.replace('<instance_geometry', '<instance_controller').replace(
        '</instance_geometry>', '</instance_controller>'),
    lambda text: text.replace('<scene>', '<library_animations/><scene>'),
    lambda text: text.replace('<scene>', '<library_controllers/><scene>'),
    lambda text: text[:-12],
])
def test_collada_rejects_malformed_external_and_unsupported(tmp_path, change):
    path = dae(tmp_path, nodes='<translate>1 2 3</translate>' + INSTANCE)
    path.write_text(change(path.read_text(encoding="utf-8")), encoding="utf-8")
    with pytest.raises(ValueError):
        mesh_vertices(path)


def test_node_cycles_and_projective_matrices_rejected(tmp_path):
    libraries = '<library_nodes><node id="cycle"><instance_node url="#cycle"/></node></library_nodes>'
    with pytest.raises(ValueError):
        mesh_vertices(dae(tmp_path, nodes='<instance_node url="#cycle"/>', libraries=libraries))
    with pytest.raises(ValueError):
        mesh_vertices(dae(tmp_path, nodes='<matrix>1 0 0 0 0 1 0 0 0 0 1 0 1 0 0 1</matrix>'
                          + INSTANCE))


ASCII_STL = """solid blade
facet normal 0 0 1
outer loop
vertex 1 2 3
vertex 4 5 6
vertex 7 8 9
endloop
endfacet
endsolid blade
"""


def binary_stl(header=b"binary"):
    return header.ljust(80, b" ") + struct.pack('<I12fH', 1, 0, 0, 1,
                                             1, 2, 3, 4, 5, 6, 7, 8, 9, 0)


@pytest.mark.parametrize("data", [ASCII_STL.encode(), binary_stl(), binary_stl(b"solid blade")])
def test_ascii_and_binary_stl_with_solid_binary_header(tmp_path, data):
    path = tmp_path / "blade.stl"
    path.write_bytes(data)
    instances = mesh_vertices(path)
    assert len(instances) == 1
    assert instances[0][0] == "blade"
    np.testing.assert_array_equal(instances[0][1], [[1, 2, 3], [4, 5, 6], [7, 8, 9]])
    assert not instances[0][1].flags.writeable


@pytest.mark.parametrize("data", [
    b"", b"solid empty\nendsolid empty\n", binary_stl()[:-1], binary_stl() + b"x",
    binary_stl(b"solid blade")[:-1], binary_stl().replace(struct.pack('<I', 1), struct.pack('<I', 2), 1),
    ASCII_STL.replace('vertex 7 8 9\n', '').encode(),
    ASCII_STL.replace('endfacet', '').encode(),
    ASCII_STL.replace('endsolid blade', 'endsolid wrong').encode(),
    ASCII_STL.replace('1 2 3', 'nan 2 3').encode(),
    ASCII_STL.replace('0 0 1', '0 0 inf').encode(),
    ASCII_STL.encode() + b"garbage",
    b"x" * 83,
])
def test_stl_rejects_truncation_nonfinite_and_malformed_grammar(tmp_path, data):
    path = tmp_path / "invalid.stl"
    path.write_bytes(data)
    with pytest.raises(ValueError):
        mesh_vertices(path)


def test_binary_stl_rejects_nonfinite_normal_and_position(tmp_path):
    path = tmp_path / "invalid.stl"
    for offset in (84, 96):
        data = bytearray(binary_stl())
        struct.pack_into('<f', data, offset, float('nan'))
        path.write_bytes(data)
        with pytest.raises(ValueError):
            mesh_vertices(path)


def test_xml_entities_duplicate_ids_empty_scene_and_file_budget_fail_closed(tmp_path):
    path = dae(tmp_path)
    path.write_text('<!DOCTYPE COLLADA [<!ENTITY x "1">]>' + path.read_text(encoding="utf-8"),
                    encoding="utf-8")
    with pytest.raises(ValueError):
        mesh_vertices(path)
    with pytest.raises(ValueError):
        mesh_vertices(dae(tmp_path, nodes=""))
    with pytest.raises(ValueError):
        mesh_vertices(dae(tmp_path, nodes=INSTANCE + '<node id="root"/>'))
    with pytest.raises(ValueError):
        mesh_vertices(dae(tmp_path, scene=""))
    large = tmp_path / "too-large.stl"
    with large.open('wb') as stream:
        stream.truncate(64 * 1024 * 1024 + 1)
    with pytest.raises(ValueError):
        mesh_vertices(large)


def test_collada_xml_depth_budget_is_enforced(tmp_path):
    nodes = '<node>' * 130 + INSTANCE + '</node>' * 130
    with pytest.raises(ValueError):
        mesh_vertices(dae(tmp_path, nodes=nodes))


def test_unknown_extension_and_missing_file_rejected(tmp_path):
    path = tmp_path / "mesh.obj"
    path.write_text('v 0 0 0', encoding="ascii")
    with pytest.raises(ValueError):
        mesh_vertices(path)
    with pytest.raises(ValueError):
        mesh_vertices(tmp_path / "missing.stl")
