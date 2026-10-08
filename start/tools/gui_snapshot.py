#!/usr/bin/env python3
"""Read Qt accessibility state; optionally exercise its landing button."""
import argparse
import json
from pathlib import Path
import pyatspi


def widgets():
    # Desktop retains registrations from terminated Qt applications. Bound
    # their D-Bus queries so a dead application cannot stall a live audit.
    pyatspi.Accessibility.setTimeout(500, 2000)
    desktop=pyatspi.Registry.getDesktop(0)
    apps=[]
    for candidate in desktop:
        try:
            if candidate.name=='drone_operator_gui':apps.append(candidate)
        except Exception:
            continue
    # Qt can register an empty application object before activating its
    # populated AT-SPI tree. Both objects refer to the same live process.
    if not apps:
        raise RuntimeError('Operator GUI application missing')
    pids={a.get_process_id() for a in apps}
    if len(pids)!=1:
        raise RuntimeError('Expected one operator GUI process, got '+str(len(pids)))
    app=max(apps,key=lambda a:a.childCount)
    if app.childCount==0:
        raise RuntimeError('Operator GUI accessibility tree empty')
    result=[];pending=[app];count=0
    while pending:
        node=pending.pop();count+=1
        if count>10000:raise RuntimeError('Accessibility tree too large')
        try:
            states=node.getState()
            result.append({'name':node.name,'role':node.getRoleName(),
                           'enabled':states.contains(pyatspi.STATE_ENABLED),
                           'sensitive':states.contains(pyatspi.STATE_SENSITIVE),
                           'showing':states.contains(pyatspi.STATE_SHOWING),'object':node})
            pending.extend(list(node))
        except Exception:
            continue
    return result


def snapshot(check=None):
    entries=widgets()
    buttons={e['name']:e for e in entries if e['role']=='push button'}
    result={'widgets':[{k:v for k,v in e.items() if k!='object'} for e in entries],
            'accessible':bool(buttons),'passed':None,'check':check}
    if check=='protection':
        disabled=['PX4 解锁 ARM','起飞','悬停','取消目标']
        required=disabled+['一键降落']
        missing=[n for n in required if n not in buttons]
        result['missing_buttons']=missing
        result['passed']=not missing and all(not buttons[n]['enabled'] for n in disabled) and buttons['一键降落']['enabled']
    elif check=='fcu_stale':
        required=['PX4 解锁 ARM','起飞','悬停','取消目标','一键降落']
        missing=[n for n in required if n not in buttons]
        result['missing_buttons']=missing
        result['passed']=not missing and all(not buttons[n]['enabled'] for n in required)
    elif check=='camera_stale':
        names=[e['name'] for e in entries]
        result['passed']=any('前断流' in n and '下断流' in n for n in names)
    return result


def click_land():
    matches=[e for e in widgets() if e['name']=='一键降落' and e['role']=='push button']
    if len(matches)!=1 or not matches[0]['enabled']:
        raise RuntimeError('Landing button missing or disabled')
    action=matches[0]['object'].queryAction()
    names=[action.getName(i) for i in range(action.nActions)]
    for i,name in enumerate(names):
        if name.casefold() in ('press','click'):
            if not action.doAction(i):raise RuntimeError('GUI rejected landing action')
            return {'activated':True,'action':action.getName(i)}
    raise RuntimeError('No accessible landing action: '+repr(names))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output');parser.add_argument('--check',choices=['protection','fcu_stale','camera_stale'])
    parser.add_argument('--click-land',action='store_true');args=parser.parse_args()
    try:
        result=snapshot(args.check)
        if args.click_land:result['landing_action']=click_land()
    except Exception as exc:
        result={'passed':False,'accessible':False,'error':str(exc)}
    Path(args.output).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='widgets'},ensure_ascii=False))
    raise SystemExit(0 if result.get('accessible') and result.get('passed') is not False else 1)
