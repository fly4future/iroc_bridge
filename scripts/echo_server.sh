#!/usr/bin/env bash

# from https://stackoverflow.com/questions/5725430/http-test-server-accepting-get-post-requests
while true; do
  resp=$"$(date): received"
  len="$(printf '%s' "$resp" | wc -c)"
  printf "HTTP/1.1 200 OK\r\nContent-Length: $len\r\n\r\n${resp}\n" | nc -Nl 8000; printf "\n\n"
done
