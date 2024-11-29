[venv 가상환경 만들기]
python3 -m venv iot(가상환경 이름)

-가상환경 활성화
source iot/bin/activate


-패키지 설치
pip install grpcio grpcio-tools
[참고] pip 업데이트 하세요 최신(24.11.29 V24.3.1)

[프로토 파일 컴파일법]
python -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. greet.proto


[greet.proto 코드]
syntax = "proto3";

service Greeter {
  rpc SayHello (HelloRequest) returns (HelloReply);
}

message HelloRequest {
  string name = 1;
}

message HelloReply {
  string message = 1;
}


[ server.py ]
import grpc
from concurrent import futures
import greet_pb2
import greet_pb2_grpc

class GreeterServicer(greet_pb2_grpc.GreeterServicer):
    def SayHello(self, request, context):
        return greet_pb2.HelloReply(message=f"Hello, {request.name}!")

def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    greet_pb2_grpc.add_GreeterServicer_to_server(GreeterServicer(), server)
    server.add_insecure_port('[::]:50051')
    print("Server started on port 50051")
    server.start()
    server.wait_for_termination()

if __name__ == "__main__":
    serve()


[client.py]
import grpc
import greet_pb2
import greet_pb2_grpc

def run():
    with grpc.insecure_channel('localhost:50051') as channel:
        stub = greet_pb2_grpc.GreeterStub(channel)
        response = stub.SayHello(greet_pb2.HelloRequest(name="World"))
        print("Client received:", response.message)

if __name__ == "__main__":
    run()








