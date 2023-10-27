#include "visualization_sender.h"


Sender::Sender(/* args */)
{
    address_ = "localhost:50051";
    channel_ptr_ = grpc::CreateChannel(address_,grpc::InsecureChannelCredentials());
    stub_ptr_ = Visualizer::NewStub(channel_ptr_);    
}

void Sender::send(const visualization::Frame &msg){
    ClientContext context;
    visualization::Reply reply;
    Status status = stub_ptr_->DataSend(&context,msg,&reply);
    if(status.ok()){
        std::cout<<"Greeter client received: ture. "<<std::endl;
        return ;
    }
    else{
        std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
        return ;
    }
    return ;
}

