#!/bin/sh

# Generate all descriptor files that have generators.

if [ $# -eq 0 ]
then
  groups=(CANLEVER CANSERVO CANMIO CANPAN CANCMD CANSOL)
else
  groups="$@"
fi


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

for g in ${groups[@]}
do
  case $g in
  CANLEVER)
    $gen_dir/generate_CANLEVER.sh | writeIfUpdated "$merg_dir"/CANLEVER-0D20-1a.json
    ;;

  CANSERVO)
    # CANMIO-SVO - Default firmware shipped with CANMIO boards.
    python $gen_dir/generate_CANMIO-SVO.py | writeIfUpdated "$merg_dir"/CANMIO-SVO-A532-4S.json
    python $gen_dir/generate_CANMIO-SVO.py -t CANSERVO8C | writeIfUpdated "$merg_dir"/CANSERVO8C-A513-4S.json
    ;;

  CANMIO)
    # Default processor is PIC18F26K80
    python $gen_dir/generate_CANMIO.py -v 3a | writeIfUpdated "$merg_dir"/CANMIO-A520-3a.json
    python $gen_dir/generate_CANMIO.py -v 3c | writeIfUpdated "$merg_dir"/CANMIO-A520-3c.json
    python $gen_dir/generate_CANMIO.py -v 3d | writeIfUpdated "$merg_dir"/CANMIO-A520-3d.json
    python $gen_dir/generate_CANMIO.py -v 3e | writeIfUpdated "$merg_dir"/CANMIO-A520-3e.json
    python $gen_dir/generate_CANMIO.py -v 4a | writeIfUpdated "$merg_dir"/CANMIO-A520-4a.json
    python $gen_dir/generate_CANMIO.py -v 4b | writeIfUpdated "$merg_dir"/CANMIO-A520-4b.json
    python $gen_dir/generate_CANMIO.py -v 4c | writeIfUpdated "$merg_dir"/CANMIO-A520-4c.json
    
    # Processor P18F27Q83
    python $gen_dir/generate_CANMIO.py -p23 -v 4a | writeIfUpdated "$merg_dir"/CANMIO-A520-4a--P23.json
    python $gen_dir/generate_CANMIO.py -p23 -v 4b | writeIfUpdated "$merg_dir"/CANMIO-A520-4b--P23.json
    python $gen_dir/generate_CANMIO.py -p23 -v 4c | writeIfUpdated "$merg_dir"/CANMIO-A520-4c--P23.json
    
    # Extended CANMIO
    # Default processor PIC18F46K80
    python $gen_dir/generate_CANMIO.py -t XIO -v 3e | writeIfUpdated "$merg_dir"/CANXIO-A540-3e.json
    python $gen_dir/generate_CANMIO.py -t XIO -v 4a | writeIfUpdated "$merg_dir"/CANXIO-A540-4a.json
    python $gen_dir/generate_CANMIO.py -t XIO -v 4b | writeIfUpdated "$merg_dir"/CANXIO-A540-4b.json
    python $gen_dir/generate_CANMIO.py -t XIO -v 4c | writeIfUpdated "$merg_dir"/CANXIO-A540-4c.json
    python $gen_dir/generate_CANMIO.py -t XIO -p22 -v 4a | writeIfUpdated "$merg_dir"/CANXIO-A540-4a--P21.json
    python $gen_dir/generate_CANMIO.py -t XIO -p22 -v 4b | writeIfUpdated "$merg_dir"/CANXIO-A540-4b--P21.json
    python $gen_dir/generate_CANMIO.py -t XIO -p22 -v 4c | writeIfUpdated "$merg_dir"/CANXIO-A540-4c--P21.json
    ;;

  CANPAN)
    python $gen_dir/generate_CANPAN.py -v 1Y | writeIfUpdated "$merg_dir"/CANPAN-A51D-1Y.json
    python $gen_dir/generate_CANPAN.py -v 4C | writeIfUpdated "$merg_dir"/CANPAN-A51D-4C.json
    python $gen_dir/generate_CANPAN.py -p23 -v 5a | writeIfUpdated "$merg_dir"/CANPAN-A51D-5a.json
    ;;

  CANCMD)
    python $gen_dir/generate_CANCMD.py -t CMD -v 4d | writeIfUpdated "$merg_dir"/CANCMD-A50A-4d.json
    python $gen_dir/generate_CANCMD.py -t CMD -v 4f | writeIfUpdated "$merg_dir"/CANCMD-A50A-4f.json
    
    python $gen_dir/generate_CANCMD.py -t CSB -v 4d | writeIfUpdated "$merg_dir"/CANCSB-A537-4d.json
    python $gen_dir/generate_CANCMD.py -t CSB -v 4f | writeIfUpdated "$merg_dir"/CANCSB-A537-4f.json
    
    python $gen_dir/generate_CANCMD.py -t CMDB -v 4f | writeIfUpdated "$merg_dir"/CANCMDB-A553-4f.json
    ;;

  CANSOL)
    python $gen_dir/generate_CANSOL.py -t ACC4 -v 2N | writeIfUpdated "$merg_dir"/CANACC4_2-A508-2N.json
    python $gen_dir/generate_CANSOL.py -t ACC4 -v 2Q | writeIfUpdated "$merg_dir"/CANACC4_2-A508-2Q.json

    python $gen_dir/generate_CANSOL.py -t SOL -v 1B | writeIfUpdated "$merg_dir"/CANSOL-A522-1B.json
    ;;

  *)
    echo "Unknown module group: $g"
  esac
done    