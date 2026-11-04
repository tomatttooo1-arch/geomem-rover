// Offline forced-candidate diagnostic; no online publication or global alignment.
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Registration.h>
#include <rtabmap/core/RegistrationInfo.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/ULogger.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
#include <memory>
#include <set>
#include <sstream>

using namespace rtabmap;
namespace fs=std::filesystem;
static std::string escaped(const std::string &s) {
    std::ostringstream o;o<<'"';
    for(unsigned char c:s) {
        if(c=='"'||c=='\\')o<<'\\'<<c;
        else if(c=='\n')o<<"\\n";
        else if(c=='\r')o<<"\\r";
        else if(c=='\t')o<<"\\t";
        else if(c<32)o<<'?';else o<<c;
    }
    o<<'"';return o.str();
}
static SensorData variant(const SensorData &data,int rgbMode,bool zeroDepth,int id) {
    if(data.cameraModels().size()!=1)throw std::runtime_error("single calibrated camera required");
    cv::Mat rgb=data.imageRaw().clone(),depth=data.depthRaw().clone();
    if(rgbMode==1)cv::GaussianBlur(rgb,rgb,cv::Size(31,31),8.);
    if(rgbMode==2)rgb.setTo(0);
    if(zeroDepth)depth.setTo(0);
    return SensorData(rgb,depth,data.cameraModels()[0],id,data.stamp());
}

int main(int argc,char **argv) {
    if(argc!=4){std::cerr<<"DB selection.tsv new-output-directory\n";return 2;}
    fs::path out(argv[3]);if(fs::exists(out))throw std::runtime_error("output exists");
    fs::create_directories(out/"prior-images");
    ULogger::setLevel(ULogger::kFatal);cv::setNumThreads(1);
    ParametersMap parameters=Parameters::getDefaultParameters();
    std::ofstream parameterFile(out/"registration-parameters.json");parameterFile<<"{\n";
    bool first=true;for(const auto &p:parameters){if(!first)parameterFile<<",\n";first=false;parameterFile<<escaped(p.first)<<":"<<escaped(p.second);}parameterFile<<"\n}\n";
    std::unique_ptr<DBDriver> db(DBDriver::create());
    if(!db->openConnection(argv[1],false,true))throw std::runtime_error("cannot read prior");
    std::set<int> ids;db->getAllNodeIds(ids);if(ids.size()!=39)throw std::runtime_error("expected frozen 39-node prior");
    std::map<int,SensorData> prior;
    std::ofstream calibrationFile(out/"prior-calibration.jsonl");
    for(int id:ids){
        SensorData data;db->getNodeData(id,data,true,false,false,false);data.uncompressData();
        if(data.imageRaw().empty()||data.depthRaw().empty()||data.cameraModels().size()!=1)throw std::runtime_error("missing map image/depth/calibration");
        const auto &c=data.cameraModels()[0];
        calibrationFile<<"{\"id\":"<<id<<",\"fx\":"<<c.fx()<<",\"fy\":"<<c.fy()<<",\"cx\":"<<c.cx()<<",\"cy\":"<<c.cy()<<",\"local_transform\":"<<escaped(c.localTransform().prettyPrint())<<"}\n";
        prior.emplace(id,data);cv::imwrite((out/"prior-images"/(std::to_string(id)+".png")).string(),data.imageRaw());
    }
    db->closeConnection(false);
    std::ofstream output(out/"pairs.jsonl");output<<std::setprecision(10);
    auto pair=[&](const std::string &kind,int index,int id,const SensorData &from,const SensorData &to,int rgbMode,int depthMode){
        cv::setRNGSeed(0);std::srand(0);RegistrationInfo info;
        std::unique_ptr<Registration> registration(Registration::create(parameters));
        Transform t=registration->computeTransformation(from,to,Transform(),&info);
        output<<"{\"kind\":"<<escaped(kind)<<",\"index\":"<<index<<",\"prior_id\":"<<id<<",\"rgb_variant\":"<<rgbMode<<",\"zero_query_depth\":"<<(depthMode?"true":"false")
              <<",\"accepted\":"<<(t.isNull()?"false":"true")<<",\"inliers\":"<<info.inliers<<",\"matches\":"<<info.matches<<",\"rejected_reason\":"<<escaped(info.rejectedMsg)
              <<",\"transform\":"<<(t.isNull()?"null":escaped(t.prettyPrint()))<<"}\n";output.flush();return !t.isNull();
    };
    int firstId=*ids.begin(),secondId=*std::next(ids.begin());
    bool self=pair("positive_self",-1,firstId,prior.at(firstId),variant(prior.at(firstId),0,false,100000),0,0);
    bool neighbor=pair("positive_neighbor",-1,firstId,prior.at(firstId),prior.at(secondId),0,0);
    bool black=pair("negative_black",-1,firstId,prior.at(firstId),variant(prior.at(firstId),2,false,100001),2,0);
    bool noDepth=pair("negative_no_depth_both",-1,firstId,variant(prior.at(firstId),0,true,firstId),variant(prior.at(firstId),0,true,100002),0,1);
    if(!self||!neighbor||black||noDepth){std::cerr<<"pair instrument calibration failed\n";return 3;}
    std::ifstream selection(argv[2]);if(!selection)throw std::runtime_error("missing selection");
    std::string line;
    while(std::getline(selection,line)){
        std::istringstream fields(line);std::string category,rgbPath,depthPath;int index,candidate;
        if(!(fields>>category>>index>>rgbPath>>depthPath>>candidate))throw std::runtime_error("invalid selected frame");
        cv::Mat rgb=cv::imread(rgbPath,cv::IMREAD_COLOR),depth16=cv::imread(depthPath,cv::IMREAD_UNCHANGED),depth;
        if(rgb.size()!=cv::Size(640,480)||depth16.type()!=CV_16UC1||depth16.size()!=rgb.size())throw std::runtime_error("invalid replay image");
        depth16.convertTo(depth,CV_32FC1,1./5000.);
        CameraModel model(517.3,516.5,318.6,255.3,CameraModel::opticalRotation(),0.,rgb.size());
        SensorData query(rgb,depth,model,100000+index);
        for(int id:ids)pair(category,index,id,prior.at(id),query,0,0);
        if(category=="always_matched"){
            if(!ids.count(candidate))throw std::runtime_error("positive control candidate outside prior");
            for(int blur=0;blur<2;blur++)for(int zero=0;zero<2;zero++)
                pair("quality_factor",index,candidate,prior.at(candidate),variant(query,blur,zero,100000+index),blur,zero);
        }
    }
    std::cout<<"completed calibrated forced-candidate diagnostics\n";
    return 0;
}
