#! /bin/bash

java -jar ../server.jar -l ../levels/SAsoko3_16.lvl -c "python searchclient/searchclient.py --max-memory 2048 -greedy" -g 500 -t 300
