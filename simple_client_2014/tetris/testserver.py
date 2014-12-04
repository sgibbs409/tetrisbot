#!/usr/bin/env python 

""" 
A simple echo server 
""" 

import socket 
import sys

host = '' 
port = 8189 
backlog = 5 
size = 1024 
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
s.bind((host,port)) 
s.listen(backlog) 
while 1: 
    client, address = s.accept() 
    while True:
       line = sys.stdin.readline()
       if not line:
           break
       print line
       client.send(line)
       data = client.recv(4) 
       print '"%s"\n'%data
       sys.stdout.flush()
    client.close()
