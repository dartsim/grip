#!/usr/bin/env python

import os

def fixfile(filename):
    f=open(filename)
    lines=""
    for l in f.readlines():
        if l.lower().find("3ds") != -1:
            a = l.split("\t")[:-1]
            b = l.split(" ")[:-1]
            if len(a) > len(b):
                fixed = a
		delim = "\t"
            else:
                fixed = b
		delim = " "
            fixed.append(l.split(delim)[-1].lower())
            l=delim.join(fixed)
        lines += l
        
    f.close()
    f=open(filename,'w')
    f.writelines(lines)
    #print lines



for model in os.popen('find . | grep -i 3ds | grep -v svn').readlines():
    if model.strip() != model.strip().lower():
        os.system("svn mv " + model.strip() + " " + model.strip().lower())

for f in os.popen('find . | grep -i rsdh | grep -v svn').readlines():
    fixfile(f.strip())

for f in os.popen('find . | grep -i rscene | grep -v svn').readlines():
    fixfile(f.strip())
