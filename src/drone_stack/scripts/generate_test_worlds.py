#!/usr/bin/env python3
"""Generate deterministic corridor, vertical-obstacle and blocked scenarios."""
from pathlib import Path
import copy
import xml.etree.ElementTree as ET

root=Path(__file__).resolve().parents[1]/'worlds'
base=ET.parse(root/'inspection_demo.world').getroot()
scenes={
    'corridor':[('corridor_left',(3,.1,1.5),(4,.2,3)),('corridor_right',(3,1.9,1.5),(4,.2,3))],
    '3d':[('low_barrier',(3,1,.5),(.6,1.4,1)),('overhead_beam',(4.5,1,2.3),(.5,1.4,.4))],
    'blocked':[('blocking_wall',(2.8,0,1.5),(.3,18,3))]}
for name,boxes in scenes.items():
    sdf=copy.deepcopy(base);world=sdf.find('world');world.set('name','inspection_'+name)
    for m in list(world.findall('model')):
        if m.get('name')=='pillar_a':world.remove(m)
    for label,pos,size in boxes:
        model=ET.SubElement(world,'model',name=label);ET.SubElement(model,'static').text='true'
        link=ET.SubElement(model,'link',name='obstacle');ET.SubElement(link,'pose').text=' '.join(map(str,pos))+' 0 0 0'
        for kind in ['collision','visual']:
            shape=ET.SubElement(link,kind,name=kind);box=ET.SubElement(ET.SubElement(shape,'geometry'),'box');ET.SubElement(box,'size').text=' '.join(map(str,size))
            if kind=='visual':ET.SubElement(ET.SubElement(shape,'material'),'ambient').text='.4 .6 .8 1'
    path=root/('inspection_'+name+'.world');ET.ElementTree(sdf).write(path,encoding='unicode',xml_declaration=True);print(path)
