folder="$(date +"%Y-%m-%d_%Hh")"
mkdir -p "bkp/$folder"

cp solver.cpp -u -t "bkp/$folder"
cp analyzer.cpp -u -t "bkp/$folder"
cp img -r -t "bkp/$folder"
cp data -r -t "bkp/$folder"
cp logs -r -t "bkp/$folder"
cp outputs -r -t "bkp/$folder"
