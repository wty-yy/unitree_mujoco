#include "height_map.h"
#include <iostream>
#include <memory>

int main(int argc, char** argv)
{
    try {
        if(argc!=2) throw std::runtime_error("Expected flat scene path");
        char error[1024]{};
        std::unique_ptr<mjModel,decltype(&mj_deleteModel)> model(mj_loadXML(argv[1],nullptr,error,sizeof(error)),mj_deleteModel);
        if(!model) throw std::runtime_error(error);
        std::unique_ptr<mjData,decltype(&mj_deleteData)> data(mj_makeData(model.get()),mj_deleteData);
        data->qpos[2]=.8;
        const int floor=mj_name2id(model.get(),mjOBJ_GEOM,"floor");
        // Rotate the plane around Y: world surface z = -tan(angle)*x.
        const double angle=.12;
        model->geom_quat[4*floor]=std::cos(angle/2);
        model->geom_quat[4*floor+2]=std::sin(angle/2);
        std::vector<mjvGeom> points;
        for(double offset : {0.0, 0.5}) for(double yaw : {0.0,1.5707963267948966}) {
            data->qpos[3]=std::cos(yaw/2); data->qpos[6]=std::sin(yaw/2);
            mj_forward(model.get(),data.get());
            mju_quat2Mat(data->geom_xmat+9*floor, model->geom_quat+4*floor);
            const auto map=sampleHeightMap(model.get(),data.get(),&points,offset);
            if(points.size()!=187) throw std::runtime_error("Expected one visualization point per valid cell");
            if(map!=sampleHeightMap(model.get(),data.get(),nullptr,offset))
                throw std::runtime_error("Visualization changed policy heights");
            for(int x=0;x<17;++x) for(int y=0;y<11;++y) {
                const double wx=std::cos(yaw)*(-.8+offset+.1*x)-std::sin(yaw)*(-.5+.1*y);
                const double wy=std::sin(yaw)*(-.8+offset+.1*x)+std::cos(yaw)*(-.5+.1*y);
                const double expected=-std::tan(angle)*wx-.8;
                if(std::abs(map[x*11+y]-expected)>1e-5) { std::cerr << "yaw="<<yaw<<" x="<<x<<" y="<<y<<" actual="<<map[x*11+y]<<" expected="<<expected<<" base="<<data->xpos[3*mj_name2id(model.get(),mjOBJ_BODY,"pelvis")]<<"\n"; throw std::runtime_error("Grid order/yaw/relative height or robot exclusion mismatch"); }
                const auto& point=points[x*11+y];
                if(point.type!=mjGEOM_SPHERE || point.category!=mjCAT_DECOR ||
                   point.objid!=-1 || point.rgba[1]!=1.0f ||
                   std::abs(point.pos[0]-wx)>1e-5 || std::abs(point.pos[1]-wy)>1e-5 ||
                   std::abs(point.pos[2]-(-std::tan(angle)*wx+point.size[0]))>1e-5)
                    throw std::runtime_error("Terrain visualization position or decoration mismatch");
            }
        }
        data->qpos[2]=3.0;
        mj_forward(model.get(),data.get());
        mju_quat2Mat(data->geom_xmat+9*floor, model->geom_quat+4*floor);
        const auto clipped=sampleHeightMap(model.get(),data.get(),&points);
        for(int i=0;i<187;++i) {
            if(clipped[i]!=-2.0f ||
               std::abs(points[i].pos[2]-(-std::tan(angle)*points[i].pos[0]+points[i].size[0]))>1e-5)
                throw std::runtime_error("Visualization must stay on terrain when policy height is clipped");
        }
        model->geom_contype[floor]=0;model->geom_conaffinity[floor]=0;
        for(float h:sampleHeightMap(model.get(),data.get(),&points))
            if(std::isfinite(h)) throw std::runtime_error("No terrain must produce missing cells");
        if(!points.empty()) throw std::runtime_error("Missing cells must clear previous visualization points");
        std::cout<<"Height-map slope, yaw, robot exclusion, missing cells and visualization passed\n";
        return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
