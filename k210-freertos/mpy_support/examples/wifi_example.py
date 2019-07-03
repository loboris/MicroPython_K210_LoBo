
import _thread, network, time, machine, socket, gc, os

# Start the WiFi module
# Configuration for MAIX-M1 (DanDock), for other boards use rx&tx on which the module is connected

wifi = network.wifi
#Debuging can be set to on to enable detailed report of the WiFi communication
#wifi.debug(True)
wifi.start(tx=7, rx=6, ssid="<your_ssid>", password="<your_password>", wait=True)

# check the WiFi status
wifi.status()
# (89, 'Idle') should be returned


# ==== Simple examples using the sockets ====

# --- Get file from server
header = "GET /K210/test.txt HTTP/1.0\r\nHost: loboris.eu\r\nConnection: keep-alive\r\n\r\n"

s = socket.socket()
# Connect to the server, if the connection was not successful, error will be raised
s.connect(("loboris.eu", 80))
s.write(header)
# Read the content returned by server
res = s.read()
#Print the returned content, in this example it will contain
#headers and text returnedby the web server
print(res.decode())

s.close()


# --- Use 'requests' module to communicate with the web server
# Secure connection can be used, just replace 'http' with 'https'

requests = network.requests
#requests debugging can be turnoed on
#requests.debug(True)

# Get file to server
res = requests.get('http://loboris.eu/K210/test.txt')
# A tuple is returned: (status, headers, content)
#Get the status
res[0]
#Print headers
print(res[1])
#Print the content, it is returned as a bytearray
print(res[2].decode())

# Get file from server into the local file
res = requests.get('http://loboris.eu/K210/test.txt', file='/flash/webtest.txt')
print(res[2])
# returns: Saved to file '/flash/webtest.txt', size=1062


# post some parameters

p4=b'\x00\x01\x02\x03\x7d\x7e\xf8\xf9\xfa'
p={"p1" : "Hi, this is a test string", "p2" : 123, "p3" : 5.4321}
p1={"p1" : "Hi, this is a test string", "p2" : 123, "p3" : 5.4321, "p4" : p4}
p2={"p1" : "Hi, this is a test string", "p2" : 123, "p3" : 5.4321, "p4" : p4, "p5": "/flash/webtest.txt"}

res = requests.get('http://loboris.eu/K210/test.php?par1=123&par2=test')

res=requests.post('http://loboris.eu/K210/test.php', p)

res=requests.post('http://loboris.eu/K210/test.php', p1, multipart=True)

res=requests.post('http://loboris.eu/K210/test.php', p2, multipart=True)

