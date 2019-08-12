#!/usr/bin/env bash

rsync -avz --exclude '.git' --filter=':- .gitignore' ~/MAVProxy nvidia@poliwhirl:~/slovak/

while true; do
  inotifywait -r -e modify,create,delete ~/MAVProxy
  rsync -avz --exclude '.git' --filter=':- .gitignore' ~/MAVProxy nvidia@poliwhirl:~/slovak/
done

