import os

def remove_position(file_name, position):
    pre = f[:position]
    suf = f[position+1:]
    return pre + suf

for dpath, dnames, fnames in os.walk('../teach_images/'):
    for f in fnames:
        os.chdir(dpath)
        if f.startswith('SQU'):
            os.rename(f, remove_position(f, 3))
        '''
        if f.startswith('CR'):
            os.rename(f, f.replace('CR', 'CRO'))
        '''