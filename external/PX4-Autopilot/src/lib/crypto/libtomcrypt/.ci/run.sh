#!/bin/bash

# output version
bash .ci/printinfo.sh

bash .ci/build.sh " $1" "$2" "$3" "$4" "$5"
if [ -a testok.txt ] && [ -f testok.txt ]; then
   echo
else
   echo
   echo "Test failed"
   exit 1
fi

rm -f testok.txt
bash .ci/build.sh " $1" "$2" "$3 LTC_DEBUG=1" "$4" "$5"
if [ -a testok.txt ] && [ -f testok.txt ]; then
   echo
else
   echo
   echo "Test failed"
   exit 1
fi

rm -f testok.txt
bash .ci/build.sh " $1" "$2 -O2" "$3 IGNORE_SPEED=1" "$4" "$5"
if [ -a testok.txt ] && [ -f testok.txt ]; then
   echo
else
   echo
   echo "Test failed"
   exit 1
fi

rm -f testok.txt
bash .ci/build.sh " $1" "$2" "$3 IGNORE_SPEED=1 LTC_SMALL=1" "$4" "$5"
if [ -a testok.txt ] && [ -f testok.txt ]; then
   echo
else
   echo
   echo "Test failed"
   exit 1
fi

exit 0

# ref:         px4, develop
# git commit:  673f5ce29015a9bba3c96792920a10601b5b0718
# commit time: 2021-06-04 18:51:41 +0200
