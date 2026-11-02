from types import SimpleNamespace as NS
import pytest


def test_saved_integer_zero_yaw_matches_identical_live_grid(saver, grid, tmp_path):
    from lite3_odom_tf_adapter.map_identity import grid_signature, read_bundle
    grid.info.origin.orientation.z = 0.0
    grid.info.origin.orientation.w = 1.0
    prefix = tmp_path / 'zero_yaw'
    saver.save_grid(grid, prefix)
    assert read_bundle(prefix)['grid_signature'] == grid_signature(grid)


def test_review_binds_exact_files_and_tampering_is_rejected(saver, grid, tmp_path):
    from lite3_odom_tf_adapter.map_identity import read_bundle, review_bundle
    prefix=tmp_path/'room';saver.save_grid(grid,prefix)
    bundle=read_bundle(prefix)
    with pytest.raises(ValueError,match='not explicitly reviewed'):read_bundle(prefix,require_reviewed=True)
    review={'map_id':bundle['map_id'],'reviewed_by':'SYNTHETIC TEST REVIEWER','notes':'Test only',
            'capture_report_sha256':'a'*64,'source_sha256':'b'*64,'config_sha256':'c'*64,
            'operating_region_cells':[0,0,3,2],'geometry_measured':True,'timing_reviewed':True,
            'measurements':[{'measured_m':1,'map_m':1},{'measured_m':2,'map_m':2}],
            'angle_measurement':{'measured_degrees':90,'map_degrees':90}}
    reviewed=review_bundle(prefix,review)
    assert read_bundle(prefix,require_reviewed=True)['map_id']==bundle['map_id']==reviewed['map_id']
    prefix.with_suffix('.pgm').write_bytes(b'corruption')
    with pytest.raises(ValueError,match='checksum'):read_bundle(prefix)


def test_stale_review_wrong_geometry_and_synthetic_maps_cannot_authorize(saver, grid, tmp_path):
    from lite3_odom_tf_adapter.map_identity import read_bundle, review_bundle
    from lite3_odom_tf_adapter.map_bundle import load_manifest,write_manifest
    prefix=tmp_path/'room';saver.save_grid(grid,prefix)
    manifest=load_manifest(prefix);manifest['synthetic']=True;manifest['quality']['approval_state']='reviewed'
    write_manifest(prefix,manifest)
    with pytest.raises(ValueError,match='Synthetic'):read_bundle(prefix,require_reviewed=True)
    with pytest.raises(ValueError,match='Synthetic'):review_bundle(prefix,{})


def test_legacy_map_can_be_used_for_localization_but_never_reviewed_by_default(saver, grid, tmp_path):
    from lite3_odom_tf_adapter.map_identity import read_bundle
    from lite3_odom_tf_adapter.map_bundle import manifest_path
    prefix = tmp_path/'old_room'
    saver.save_grid(grid, prefix)
    manifest_path(prefix).unlink()
    legacy = read_bundle(prefix, allow_legacy=True)
    assert legacy['manifest']['legacy_unverified'] is True
    assert legacy['approval_state'] == 'experimental'
    assert legacy['map_id']
    with pytest.raises(ValueError, match='integrity manifest'):
        read_bundle(prefix, require_reviewed=True)
    manifest_path(prefix).write_text('{broken json')
    with pytest.raises(ValueError, match='integrity manifest'):
        read_bundle(prefix, allow_legacy=True)


def test_legacy_map_catalog_exposes_identity_for_pose_only(api, api_module, saver, grid):
    from lite3_odom_tf_adapter.map_bundle import manifest_path
    prefix = api.maps_dir/'old_room'
    saver.save_grid(grid, prefix)
    manifest_path(prefix).unlink()
    item = next(entry for entry in api.list_maps() if entry['name'] == 'old_room')
    assert item['integrity'] == 'legacy_unverified'
    assert item['approval_state'] == 'experimental'
    api._navigation_map_name = 'old_room'
    api._navigation_identity = item['map_id']
    assert api._assert_active_map('old_room', item['map_id'], require_reviewed=False)['map_id'] == item['map_id']
    with pytest.raises(api_module.ApiError, match='integrity manifest'):
        api._assert_active_map('old_room', item['map_id'])
