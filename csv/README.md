# Shell code to merge the csv files

Move to either axon or acxon based folders


$     cat ./test/rovers/problem_instances_1-2_with_specificity_1.csv | head -n1 > merged.csv

$     for f in ./test/*/*.csv; do cat "`pwd`/$f" | tail -n +2 >> merged.csv; done 

