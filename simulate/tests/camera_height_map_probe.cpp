#include "camera_height_map.h"
#include <chrono>
#include <iomanip>
#include <iostream>

void array(const std::vector<float>& values) {
    std::cout<<'[';bool first=true;
    for(float v:values){if(!first)std::cout<<',';first=false;if(std::isfinite(v))std::cout<<v;else std::cout<<"null";}
    std::cout<<']';
}
int main(int argc,char**argv) {
    if(argc!=2)return 2;
    auto request=YAML::LoadFile(argv[1]);char error[1024];
    std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML(request["scene"].as<std::string>().c_str(),nullptr,error,sizeof(error)),mj_deleteModel);
    if(!model)throw std::runtime_error(error);
    std::unique_ptr<mjData,decltype(&mj_deleteData)> data(mj_makeData(model.get()),mj_deleteData);
    CameraHeightMapConfig cfg;cfg.load(request["config"]);
    CameraHeightMap map(model.get(),request["width"].as<int>(17),request["origin_x"].as<double>(-.5),cfg);
    std::cout<<std::setprecision(10)<<'[';bool first=true;
    for(const auto& pose:request["poses"]) {
        auto q=pose["qpos"].as<std::vector<double>>();std::copy(q.begin(),q.end(),data->qpos);
        data->time=pose["time"].as<double>();mj_forward(model.get(),data.get());
        auto start=std::chrono::steady_clock::now();bool updated=map.update(data.get());
        double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
        std::vector<CameraHeightMap::DebugPoint> points;std::vector<CameraHeightMap::DebugLine> lines;
        if(request["debug"].as<bool>(false))map.debug(data.get(),points,lines);
        if(!first)std::cout<<',';first=false;
        std::cout<<"{\"updated\":"<<(updated?"true":"false")<<",\"frame_count\":"<<map.frame_count()<<",\"delay\":"<<map.delay()<<",\"full_view\":"<<(map.full_view()?"true":"false")<<",\"ms\":"<<ms<<",\"elevation\":";
        array(map.output().elevation);std::cout<<",\"missing\":";array(map.output().missing);
        std::cout<<",\"memory\":";array(map.memory());std::cout<<",\"corruption\":";array(map.corruption());
        std::cout<<",\"hits\":[";bool first_hit=true;for(const auto& h:map.hits()){if(!first_hit)std::cout<<',';first_hit=false;std::cout<<'['<<h.x()<<','<<h.y()<<','<<h.z()<<']';}
        std::cout<<"],\"points\":[";bool first_point=true;for(const auto& p:points){if(!first_point)std::cout<<',';first_point=false;std::cout<<'['<<p.position.x()<<','<<p.position.y()<<','<<p.position.z()<<','<<p.layer<<']';}
        std::cout<<"],\"lines\":[";bool first_line=true;for(const auto& l:lines){if(!first_line)std::cout<<',';first_line=false;std::cout<<'['<<l.from.x()<<','<<l.from.y()<<','<<l.from.z()<<','<<l.to.x()<<','<<l.to.y()<<','<<l.to.z()<<','<<l.layer<<']';}
        std::cout<<"]}";
    }
    std::cout<<"]\n";
}
