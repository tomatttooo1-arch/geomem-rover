"""Read-only inventory of candidate recording files and SQLite table schemas."""
import argparse
import json
from pathlib import Path
import sqlite3
import subprocess


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--root',action='append',required=True);ap.add_argument('--out',required=True)
    a=ap.parse_args();out=Path(a.out)
    if out.exists():ap.error('output must not exist')
    patterns=['metadata.yaml','*.db3','*.mcap','*.bag','*.db','*.tar.gz','*.tgz']
    cmd=['rg','--files','--hidden','--no-ignore']+a.root
    for p in patterns:cmd+=['-g',p]
    for p in ['!**/build/**','!**/.git/**','!**/node_modules/**']:cmd+=['-g',p]
    result=subprocess.run(cmd,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    if result.returncode not in [0,1]:raise RuntimeError(result.stderr)
    rows=[]
    for name in result.stdout.splitlines():
        p=Path(name);r=dict(path=str(p),size=p.stat().st_size)
        if p.suffix=='.db':
            try:
                conn=sqlite3.connect(p.resolve().as_uri()+'?mode=ro',uri=True)
                r['tables']=[row[0] for row in conn.execute("SELECT name FROM sqlite_master WHERE type='table'")]
                r['rosbag_topics_table']='topics' in r['tables'];conn.close()
            except sqlite3.Error as error:r['error']=str(error)
        else:r['kind']='archive_not_expanded' if p.name.endswith(('.tar.gz','.tgz')) else 'rosbag_candidate'
        rows.append(r)
    out.write_text(json.dumps(dict(command=cmd,roots=a.root,files=rows,
        caveat='Archives are listed but not recursively expanded; unreadable SQLite files remain unclassified.'),indent=2)+'\n')
    print('candidate files',len(rows),'unclassified',sum('error' in r for r in rows))


if __name__=='__main__':main()
