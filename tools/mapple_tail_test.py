#!/usr/bin/env python3
"""The root partition's tail, single, split and development layouts: every area the kernel, the loader and the
installer place there, from their own sources, must lie inside the partition and overlap no other. Prints the map.

  tools/mapple_tail_test.py        (MAPPLE_BUILD, MAPPLE_LOADER: other checkouts of mapple_build and mapple_loader)
"""
import os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
TOP = os.path.normpath(os.path.join(HERE, '..', '..', '..', '..'))      # the manifest's checkout
BUILD = os.environ.get('MAPPLE_BUILD', os.path.join(TOP, 'build', 'mapple'))
LOADER = os.environ.get('MAPPLE_LOADER', os.path.join(TOP, 'mapple', 'loader'))
MiB = 1 << 20


def read(path):
    if not os.path.exists(path):
        sys.exit('mapple_tail_test: no %s (MAPPLE_BUILD / MAPPLE_LOADER)' % path)
    return open(path).read()


def define(text, name):
    m = re.search(r'#define\s+%s\s+(.+?)\s*(/\*.*)?$' % name, text, re.M)
    if not m:
        sys.exit('mapple_tail_test: no #define %s' % name)
    return m.group(1)


def value(expr, names):
    expr = re.sub(r'(\d+)u?(ll|ull|ULL|LL)?\b', r'\1', expr)
    return eval(expr, {}, names)


ufs = read(os.path.join(HERE, '..', 'bsd', 'dev', 'mtk_ufs.c'))
k = {'UFS_BLOCK': 4096}
k['UFS_BLOCK'] = value(define(ufs, 'UFS_BLOCK'), k)
for n in ('BOOTREC_TAIL_BLOCKS', 'MAPPLE_LOG_SIZE', 'MAPPLE_LOG_BLOCKS'):
    k[n] = value(define(ufs, n), k)
m = re.search(r'uint64_t base = su\.first_block \+ su\.nblocks((?:\s*-\s*\w+)+);', ufs)
if not m:
    sys.exit('mapple_tail_test: the log thread\'s base not found')
log_from_end = sum(k[t] for t in re.findall(r'\w+', m.group(1))) * k['UFS_BLOCK']

inst = read(os.path.join(BUILD, 'installer', 'mapple-install.c'))
ih = read(os.path.join(BUILD, 'installer', 'mapple-install.h'))
i = {'MiB': MiB}
i['AREAS'] = value(define(ih, 'AREAS'), i)
regions = {name: (value(a, i), value(b, i))
           for name, a, b in re.findall(r'\{\s*"(\w+)",\s*([^,{}]+),\s*([^,{}]+)\}', inst)}

root = read(os.path.join(LOADER, 'mtk', 'loader', 'mtkroot.h'))
rec_h = read(os.path.join(LOADER, 'shim', 'bootrec.h'))
fw_from_end = value(define(root, 'TAIL_FW_FROM_END'), {})
fw_size = value(define(root, 'TAIL_FW_SIZE'), {})
room = value(define(root, 'ROOT_ROOM_MIB'), {}) * MiB

fails = []
def check(cond, what):
    if not cond:
        fails.append(what)

# one place for each number: the loader and the installer agree with each other and with the kernel
check(re.search(r'bootrec_volume_clear\(base\)', ufs), 'the log thread checks the volume before writing')
check(value(define(rec_h, 'BOOTREC_TAIL_BLOCKS'), {}) == k['BOOTREC_TAIL_BLOCKS'], 'record: loader vs kernel')
check(regions.get('firmware') == (fw_from_end, fw_size), 'firmware region: installer vs loader')
check(room == i['AREAS'], 'room after the volume: loader vs installer')
check(regions.get('log', (0, 0))[0] - regions['log'][1] == k['BOOTREC_TAIL_BLOCKS'] * k['UFS_BLOCK'],
      'the installer\'s log region ends where the record\'s MiB starts')


def layout(name, part, vol_end):
    """(name, start, end) of each area for a partition of part bytes whose HFS+ volume ends at vol_end"""
    end = part
    rec = end - k['BOOTREC_TAIL_BLOCKS'] * k['UFS_BLOCK']
    a = [('HFS+ volume', 0, vol_end),
         ('boot-args record', rec, rec + k['UFS_BLOCK']),
         ('HFS+ alternate header (partition end)', end - 1024, end)]
    if vol_end <= end - i['AREAS']:
        a.append(('saved area header', end - i['AREAS'], end - i['AREAS'] + 4096))
        for n, (fe, sz) in regions.items():
            if sz:
                a.append(('installer ' + n, end - fe, end - fe + sz))
    log = end - log_from_end
    if vol_end <= log:                      # mtk_ufs.c's bootrec_volume_clear(base): else no log
        a.append(('kernel console log', log, log + k['MAPPLE_LOG_BLOCKS'] * k['UFS_BLOCK']))
    return a


def show(name, part, areas):
    print('%s (partition %d bytes; offsets from its end):' % (name, part))
    for n, s, e in sorted(areas, key=lambda x: x[1]):
        print('  %-40s end-%-14d .. end-%d' % (n, part - s, part - e))


# single: userdata holds the root; split: xnusys; development: the volume up to the record's MiB (this phone)
for name, part, vol_end in (('single', 28536995840, 28536995840 - 128 * MiB),
                            ('split', 23623335936, 23623335936 - 128 * MiB),
                            ('development', 23623335936, 23623335936 - MiB)):
    areas = layout(name, part, vol_end)
    show(name, part, areas)
    for n, s, e in areas:
        check(0 <= s < e <= part, '%s: %s outside the partition' % (name, n))
    for x in range(len(areas)):
        for y in range(x + 1, len(areas)):
            a, b = areas[x], areas[y]
            if a[1] < b[2] and b[1] < a[2] and {a[0], b[0]} != {'installer log', 'kernel console log'}:
                fails.append('%s: %s overlaps %s' % (name, a[0], b[0]))
    if name != 'development':
        log = [a for a in areas if a[0] == 'kernel console log']
        reg = [a for a in areas if a[0] == 'installer log']
        check(log and reg and reg[0][1] <= log[0][1] and log[0][2] <= reg[0][2],
              '%s: the kernel\'s log inside the installer\'s log region' % name)
    else:
        check(not [a for a in areas if a[0] == 'kernel console log'], 'development: no log over the volume')

for f in fails:
    print('FAIL: ' + f)
print('mapple_tail_test: %s' % ('FAILED' if fails else 'no overlaps, the numbers agree'))
sys.exit(1 if fails else 0)
