#!/bin/sh

# Generate all descriptor files that have generators.

gen_dir=`dirname $0`
wip_dir=`dirname $gen_dir`/work_in_progress
merg_dir="$wip_dir/MERG modules"
tmp_file=t.json

function writeIfUpdated()
{
  outFileName=$1
  cat > $tmp_file
  if diff -qw -I'"timestamp" *:' "$outFileName" $tmp_file >/dev/null
  then
    rm $tmp_file
    echo "No changes to $outFileName"
  else
    mv $tmp_file "$outFileName"
    echo "Generated $outFileName"
  fi
}

# CANLEVER
$gen_dir/generate_CANLEVER.sh | writeIfUpdated "$merg_dir"/CANLEVER-0D20-1a.json

# CANMIO-SVO - Default firmware shipped with CANMIO boards.
$gen_dir/generate_CANMIO-SVO.sh | writeIfUpdated "$merg_dir"/CANMIO-SVO-A532-4S.json
$gen_dir/generate_CANMIO-SVO.sh -t CANSERVO8C | writeIfUpdated "$merg_dir"/CANSERVO8C-A513-4S.json

# CANMIO
# Default processor is PIC18F26K80
$gen_dir/generate_CANMIO.sh -v 3a | writeIfUpdated "$merg_dir"/CANMIO-A520-3a.json
$gen_dir/generate_CANMIO.sh -v 3c | writeIfUpdated "$merg_dir"/CANMIO-A520-3c.json
$gen_dir/generate_CANMIO.sh -v 3d | writeIfUpdated "$merg_dir"/CANMIO-A520-3d.json
$gen_dir/generate_CANMIO.sh -v 3e | writeIfUpdated "$merg_dir"/CANMIO-A520-3e.json
$gen_dir/generate_CANMIO.sh -v 4a | writeIfUpdated "$merg_dir"/CANMIO-A520-4a.json
$gen_dir/generate_CANMIO.sh -v 4b | writeIfUpdated "$merg_dir"/CANMIO-A520-4b.json

# Processor P18F27Q83
$gen_dir/generate_CANMIO.sh -p23 -v 4a | writeIfUpdated "$merg_dir"/CANMIO-A520-4a--P23.json
$gen_dir/generate_CANMIO.sh -p23 -v 4b | writeIfUpdated "$merg_dir"/CANMIO-A520-4b--P23.json

# Extended CANMIO
# Default processor PIC18F46K80
$gen_dir/generate_CANMIO.sh -t XIO -v 3e | writeIfUpdated "$merg_dir"/CANXIO-A540-3e.json
$gen_dir/generate_CANMIO.sh -t XIO -v 4a | writeIfUpdated "$merg_dir"/CANXIO-A540-4a.json
$gen_dir/generate_CANMIO.sh -t XIO -v 4b | writeIfUpdated "$merg_dir"/CANXIO-A540-4b.json
$gen_dir/generate_CANMIO.sh -t XIO -p22 -v 4a | writeIfUpdated "$merg_dir"/CANXIO-A540-4a--P21.json
$gen_dir/generate_CANMIO.sh -t XIO -p22 -v 4b | writeIfUpdated "$merg_dir"/CANXIO-A540-4b--P21.json

# CANPAN
$gen_dir/generate_CANPAN.sh -v 1Y | writeIfUpdated "$merg_dir"/CANPAN-A51D-1Y.json
$gen_dir/generate_CANPAN.sh -v 4C | writeIfUpdated "$merg_dir"/CANPAN-A51D-4C.json

# CANCMD and successors
$gen_dir/generate_CANCMD.sh -v 4d | writeIfUpdated "$merg_dir"/CANCMD-A50A-4d.json
$gen_dir/generate_CANCMD.sh -v 4f | writeIfUpdated "$merg_dir"/CANCMD-A50A-4f.json

$gen_dir/generate_CANCMD.sh -t CSB -v 4d | writeIfUpdated "$merg_dir"/CANCSB-A537-4d.json
$gen_dir/generate_CANCMD.sh -t CSB -v 4f | writeIfUpdated "$merg_dir"/CANCSB-A537-4f.json

$gen_dir/generate_CANCMD.sh -t CMDB -v 4f | writeIfUpdated "$merg_dir"/CANCMDB-A553-4f.json
