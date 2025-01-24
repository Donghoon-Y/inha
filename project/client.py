import socket

server_address = 'localhost'
server_port = 12345

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((server_address, server_port))

name = '미나몬'
message = '안녕 서버'

request = f'{name}&&{message}'
client_socket.send(request.encode('utf-8'))

response = client_socket.recv(1024).decode('utf-8')
print(f'{name} :  {message}')
print(f'서버 : {response}\n')

client_socket.close()