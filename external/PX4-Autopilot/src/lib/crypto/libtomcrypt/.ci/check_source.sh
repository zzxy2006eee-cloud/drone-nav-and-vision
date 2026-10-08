#!/bin/bash

# output version
bash .ci/printinfo.sh

make clean > /dev/null

echo "checking..."
./helper.pl --check-all || exit 1

exit 0

# ref:         px4, develop
# git commit:  673f5ce29015a9bba3c96792920a10601b5b0718
# commit time: 2021-06-04 18:51:41 +0200
