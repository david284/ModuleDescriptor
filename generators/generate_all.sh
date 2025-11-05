#!/bin/sh

# Generate all descriptor files that have generators.

if [ $# -eq 0 ]
then
  groups=(CANINP CANSERVO CANMIO CANPAN CANSCAN CANCMD CANSOL)
  # NOTE: CANLEVER is not in the list as its generator is out of date.
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

  CANINP)
    # CANACE8C, CANACE8CMIO, CANTOTI and CANINP
    python $gen_dir/generate_CANINP.py -v 2n | writeIfUpdated "$merg_dir"/CANACE8C-A505-2n.json
    python $gen_dir/generate_CANINP.py -v 2p | writeIfUpdated "$merg_dir"/CANACE8C-A505-2p.json
    python $gen_dir/generate_CANINP.py -v 2q | writeIfUpdated "$merg_dir"/CANACE8C-A505-2q.json
    python $gen_dir/generate_CANINP.py -t MIO -v 2q | writeIfUpdated "$merg_dir"/CANACE8MIO-A521-2q.json
    python $gen_dir/generate_CANINP.py -t TOTI -v 2q | writeIfUpdated "$merg_dir"/CANTOTI-A511-2q.json
    python $gen_dir/generate_CANINP.py -t INP -v 2s | writeIfUpdated "$merg_dir"/CANINP-A53E-2s.json
    ;;
  
  CANOUT)
    # CANACC5, CANACC8, CANMIO-OUT and CANBIP-OUT
    python $gen_dir/generate_CANOUT.py -t ACC5 -v 2U | writeIfUpdated "$merg_dir"/CANACC5-A502-2U.json
    python $gen_dir/generate_CANOUT.py -t ACC5 -v 2V | writeIfUpdated "$merg_dir"/CANACC5-A502-2V.json
    python $gen_dir/generate_CANOUT.py -t ACC8 -v 2V | writeIfUpdated "$merg_dir"/CANACC8-A503-2V.json
    python $gen_dir/generate_CANOUT.py -t OUT -v 5c | writeIfUpdated "$merg_dir"/CANOUT-A53F-5c.json
    python $gen_dir/generate_CANOUT.py -t MIO -v 5b | writeIfUpdated "$merg_dir"/CANMIO-OUT-A534-5b.json
    python $gen_dir/generate_CANOUT.py -t BIP -v 5b | writeIfUpdated "$merg_dir"/CANBIP-OUT-A535-5b.json
    ;;

  CANSERVO)
    # CANMIO-SVO - Default firmware shipped with CANMIO boards.
    python $gen_dir/generate_CANSERVO.py | writeIfUpdated "$merg_dir"/CANMIO-SVO-A532-4S.json
    python $gen_dir/generate_CANSERVO.py -t CANSERVO8C | writeIfUpdated "$merg_dir"/CANSERVO8C-A513-2U.json
    python $gen_dir/generate_CANSERVO.py -t CANSERVO8C | writeIfUpdated "$merg_dir"/CANSERVO8C-A513-4h.json
    python $gen_dir/generate_CANSERVO.py -t CANSERVO8C | writeIfUpdated "$merg_dir"/CANSERVO8C-A513-4S.json
    ;;

  CANSOL)
    python $gen_dir/generate_CANSOL.py -t ACC4 -v 2Q | writeIfUpdated "$merg_dir"/CANACC4-A501-2Q.json

    python $gen_dir/generate_CANSOL.py -t ACC4 -v 2N | writeIfUpdated "$merg_dir"/CANACC4_2-A508-2N.json
    python $gen_dir/generate_CANSOL.py -t ACC4 -v 2Q | writeIfUpdated "$merg_dir"/CANACC4_2-A508-2Q.json

    python $gen_dir/generate_CANSOL.py -t SOL -v 1B | writeIfUpdated "$merg_dir"/CANSOL-A522-1B.json

    python $gen_dir/generate_CANSOL.py -t CDU -v 1c | writeIfUpdated "$merg_dir"/CANCDU-A524-1c.json
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
    python $gen_dir/generate_CANMIO.py -v 4d | writeIfUpdated "$merg_dir"/CANMIO-A520-4d.json
    
    # Processor P18F27Q83
    python $gen_dir/generate_CANMIO.py -p23 -v 4a | writeIfUpdated "$merg_dir"/CANMIO-A520-4a--P23.json
    python $gen_dir/generate_CANMIO.py -p23 -v 4b | writeIfUpdated "$merg_dir"/CANMIO-A520-4b--P23.json
    python $gen_dir/generate_CANMIO.py -p23 -v 4c | writeIfUpdated "$merg_dir"/CANMIO-A520-4c--P23.json
    python $gen_dir/generate_CANMIO.py -p23 -v 4d | writeIfUpdated "$merg_dir"/CANMIO-A520-4d--P23.json
    
    # Extended CANMIO
    # Default processor PIC18F46K80
    python $gen_dir/generate_CANMIO.py -t XIO -v 3e | writeIfUpdated "$merg_dir"/CANXIO-A540-3e.json
    python $gen_dir/generate_CANMIO.py -t XIO -v 4a | writeIfUpdated "$merg_dir"/CANXIO-A540-4a.json
    python $gen_dir/generate_CANMIO.py -t XIO -v 4b | writeIfUpdated "$merg_dir"/CANXIO-A540-4b.json
    python $gen_dir/generate_CANMIO.py -t XIO -v 4c | writeIfUpdated "$merg_dir"/CANXIO-A540-4c.json
    python $gen_dir/generate_CANMIO.py -t XIO -v 4d | writeIfUpdated "$merg_dir"/CANXIO-A540-4d.json
    python $gen_dir/generate_CANMIO.py -t XIO -p22 -v 4a | writeIfUpdated "$merg_dir"/CANXIO-A540-4a--P21.json
    python $gen_dir/generate_CANMIO.py -t XIO -p22 -v 4b | writeIfUpdated "$merg_dir"/CANXIO-A540-4b--P21.json
    python $gen_dir/generate_CANMIO.py -t XIO -p22 -v 4c | writeIfUpdated "$merg_dir"/CANXIO-A540-4c--P21.json
    python $gen_dir/generate_CANMIO.py -t XIO -p22 -v 4d | writeIfUpdated "$merg_dir"/CANXIO-A540-4d--P21.json
    ;;

  CANPAN)
    python $gen_dir/generate_CANPAN.py -v 1Y | writeIfUpdated "$merg_dir"/CANPAN-A51D-1Y.json
    python $gen_dir/generate_CANPAN.py -v 4C | writeIfUpdated "$merg_dir"/CANPAN-A51D-4C.json
    python $gen_dir/generate_CANPAN.py -p23 -v 5a | writeIfUpdated "$merg_dir"/CANPAN-A51D-5a.json
    ;;
  
  CANSCAN)
    python $gen_dir/generate_CANPAN.py -t SCAN -v 4b | writeIfUpdated "$merg_dir"/CANSCAN-A531-4b.json
    python $gen_dir/generate_CANPAN.py -t SCAN -v 4c | writeIfUpdated "$merg_dir"/CANSCAN-A531-4c.json
    ;;

  CANCMD)
    python $gen_dir/generate_CANCMD.py -t CMD -v 4d | writeIfUpdated "$merg_dir"/CANCMD-A50A-4d.json
    python $gen_dir/generate_CANCMD.py -t CMD -v 4f | writeIfUpdated "$merg_dir"/CANCMD-A50A-4f.json
    
    python $gen_dir/generate_CANCMD.py -t CSB -v 4d | writeIfUpdated "$merg_dir"/CANCSB-A537-4d.json
    python $gen_dir/generate_CANCMD.py -t CSB -v 4f | writeIfUpdated "$merg_dir"/CANCSB-A537-4f.json
    
    python $gen_dir/generate_CANCMD.py -t CMDB -v 4f | writeIfUpdated "$merg_dir"/CANCMDB-A553-4f.json
    ;;

  *)
    echo "Unknown module group: $g"
  esac
done    