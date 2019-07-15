import socket
import sys
import threading
import os
import multiprocessing
from multiprocessing import Queue
import time
import select
import subprocess


def run_project2(pr, q):
    try:
        for i in os.popen(pr):
            q.put(i)
    except:
        pass


def run_project(pr, q):
    p = subprocess.Popen(pr, shell = True, stdout=subprocess.PIPE)
    while True:
        tempout = p.stdout.read(200)
        if len(tempout) > 0:
            q.put(tempout)
    

def client_handler(s):
    response = ''
    while True:
        s.sendall("{}\n<remote:#>".format(response.rstrip()))
        cmd = ""
        while "\n" not in cmd:
            cmd += s.recv(1024)
        cmd = cmd.strip()
        response = ""
        if cmd == "exit":
            s.sendall("Bye")
            s.close()
            break
        if cmd[:2] == "cd":
            try:
                os.chdir(cmd[2:].strip())
            except:
                response = "Could not execute command"
        elif cmd.startswith("roslaunch") or cmd.startswith("rosrun"):
            q = Queue()
            p = multiprocessing.Process(target = run_project, args = (cmd, q))
            p.start()
            try:
                newcmd = ""
                while "%%%%%%" not in newcmd:
                    if not q.empty():
                        s.sendall(q.get())
                    ready = select.select([s], [], [], 0.2)
                    if ready[0]:
                        newcmd += s.recv(1024)
                    time.sleep(0.3)
            except KeyboardInterrupt:
                p.terminate()
            p.terminate()
            os.system("killall {}".format(cmd.split()[0]))
        else:
            try:
                response = os.popen(cmd).read()
            except:
                response = "Could not execute command\n"


def client(ip, port):
    s = socket.socket()
    s.connect((ip, port))
    while True:
        recv_len = 1
        response = ""
        while recv_len > 0:
            data = s.recv(4096)
            recv_len = len(data)
            response += data
            if recv_len < 4096:
                break
        sys.stdout.write(response)
        sys.stdout.flush()
        cmd = raw_input()
        s.sendall(cmd + '\n')
        print "sent {}".format(repr(cmd))
        if cmd.strip().startswith("roslaunch") or cmd.strip().startswith("rosrun"):
            try:
                while True:
                    sys.stdout.write(s.recv(1024))
                    sys.stdout.flush()
                    time.sleep(0.2)
            except KeyboardInterrupt:
                s.sendall("%%%%%%")
        if cmd.strip() == "exit":
            s.close()
            break
            
        

def server(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("0.0.0.0", port))
    s.listen(10)

    try:
        while True:
            client_socket, addr = s.accept()
        
            client_thread = threading.Thread(target = client_handler, args = (client_socket,))
            client_thread.start()
    except KeyboardInterrupt:
        s.close()


def printusage():
    print "Usage:\n\tpython serv.py server <port>      -- server mode\n\tpython serv.py client <ip> <port> -- client mode"
    
if __name__ == "__main__":
    if len(sys.argv) == 1:
        printusage()
        sys.exit(0)
    if sys.argv[1] == "server":
        server(int(sys.argv[2]))
    elif sys.argv[1] == "client":
        client(sys.argv[2], int(sys.argv[3]))
    else:
        printusage()
