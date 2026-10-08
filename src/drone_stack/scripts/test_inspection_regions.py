#!/usr/bin/env python3
import json
from pathlib import Path
import tempfile
import unittest
from inspection_region_store import RegionStore


def region(name='设备区'):
    return dict(name=name,polygon=[[0,0],[3,0],[3,2],[0,2]],altitude=1.2,spacing=.5,overlap=.2,
        speed=.5,angle_deg=-1.,entry=0,auto_spacing=False)


class StoreTests(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory();self.root=Path(self.directory.name)
        self.store=RegionStore(self.root,'a'*64,'/scene.dmap')
    def tearDown(self):self.directory.cleanup()

    def test_empty_library(self):self.assertEqual(self.store.read(),[])
    def test_unicode_and_restart_persistence(self):
        first=self.store.change('save',region())[0]
        second=RegionStore(self.root,'a'*64).read()[0]
        self.assertEqual(first,second);self.assertEqual(second['name'],'设备区')
    def test_map_isolation(self):
        self.store.change('save',region());self.assertEqual(RegionStore(self.root,'b'*64).read(),[])
    def test_duplicate_name_not_overwritten(self):
        self.store.change('save',region());old=self.store.file.read_bytes()
        with self.assertRaises(ValueError):self.store.change('save',region())
        self.assertEqual(old,self.store.file.read_bytes())
    def test_explicit_update_preserves_id(self):
        old=self.store.change('save',region())[0];data=region('新的设备区');data['id']=old['id'];data['altitude']=1.5
        new=self.store.change('update',data)[0]
        self.assertEqual(new['id'],old['id']);self.assertEqual(new['created_at'],old['created_at']);self.assertEqual(new['altitude'],1.5)
    def test_delete_only_selected(self):
        first=self.store.change('save',region())[0];self.store.change('save',region('北区'))
        remaining=self.store.change('delete',{'id':first['id']})
        self.assertEqual([r['name'] for r in remaining],['北区'])
    def test_invalid_polygon_rejected(self):
        data=region();data['polygon']=[[0,0],[2,2],[0,2],[2,0]]
        with self.assertRaises(ValueError):self.store.change('save',data)
        self.assertFalse(self.store.file.exists())
    def test_nonfinite_parameters_rejected(self):
        data=region();data['altitude']=float('nan')
        with self.assertRaises(ValueError):self.store.change('save',data)
    def test_corrupt_library_not_overwritten(self):
        self.store.file.write_text('{bad');old=self.store.file.read_bytes()
        with self.assertRaises(ValueError):self.store.change('save',region())
        self.assertEqual(old,self.store.file.read_bytes())
    def test_wrong_map_digest_rejected(self):
        self.store.change('save',region());data=json.loads(self.store.file.read_text());data['map_digest']='b'*64
        self.store.file.write_text(json.dumps(data))
        with self.assertRaises(ValueError):self.store.read()
    def test_temporary_files_cleaned(self):
        self.store.change('save',region());self.assertEqual(list(self.root.glob('*.tmp')),[])
    def test_unknown_update_id_rejected(self):
        data=region();data['id']='0'*32
        with self.assertRaises(ValueError):self.store.change('update',data)


if __name__=='__main__':unittest.main(verbosity=2)
