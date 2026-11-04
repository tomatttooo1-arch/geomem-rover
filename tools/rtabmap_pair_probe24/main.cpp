// Calibrated raw-image forced-pair diagnostic for the frozen 24-node map.
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
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>

using namespace rtabmap;
namespace fs=std::filesystem;
static std::string escaped(const std::string &s) {
    std::ostringstream o;o<<'"';for(unsigned char c:s){if(c=='"'||c=='\\')o<<'\\'<<c;
        else if(c=='\n')o<<"\\n";else if(c=='\r')o<<"\\r";else if(c=='\t')o<<"\\t";
        else if(c<32)o<<'?';else o<<c;}o<<'"';return o.str();
}
static SensorData variant(const SensorData &data,int rgbMode,bool zeroDepth,int id) {
    cv::Mat rgb=data.imageRaw().clone(),depth=data.depthRaw().clone();
    if(rgbMode==1)rgb.setTo(0);
    if(zeroDepth)depth.setTo(0);
    return SensorData(rgb,depth,data.cameraModels()[0],id,data.stamp());
}
static void matrix(std::ostream &out,const Transform &t) {
    out<<"[["<<t.r11()<<","<<t.r12()<<","<<t.r13()<<","<<t.o14()<<"],["
       <<t.r21()<<","<<t.r22()<<","<<t.r23()<<","<<t.o24()<<"],["
       <<t.r31()<<","<<t.r32()<<","<<t.r33()<<","<<t.o34()<<"],[0,0,0,1]]";
}
int main(int argc,char **argv) {
    if(argc!=4){std::cerr<<"DB selection.tsv output-directory\n";return 2;}
    fs::path out(argv[3]);if(fs::exists(out))throw std::runtime_error("output exists");
    fs::create_directories(out/"prior-images");ULogger::setLevel(ULogger::kFatal);cv::setNumThreads(1);
    ParametersMap parameters=Parameters::getDefaultParameters();
    std::ofstream parameterFile(out/"registration-parameters.json");parameterFile<<"{\n";
    bool first=true;for(const auto &p:parameters){if(!first)parameterFile<<",\n";first=false;parameterFile<<escaped(p.first)<<":"<<escaped(p.second);}parameterFile<<"\n}\n";
    std::unique_ptr<DBDriver> db(DBDriver::create());if(!db->openConnection(argv[1],false,true))throw std::runtime_error("cannot read prior");
    std::set<int> ids;db->getAllNodeIds(ids);if(ids.size()!=24)throw std::runtime_error("expected frozen 24-node prior");
    std::map<int,SensorData> prior;std::ofstream calibrationFile(out/"prior-calibration.jsonl");
    for(int id:ids){SensorData data;db->getNodeData(id,data,true,false,false,false);data.uncompressData();
        if(data.imageRaw().empty()||data.depthRaw().empty()||data.cameraModels().size()!=1)throw std::runtime_error("missing prior raw data");
        const auto &c=data.cameraModels()[0];calibrationFile<<"{\"id\":"<<id<<",\"fx\":"<<c.fx()<<",\"fy\":"<<c.fy()<<",\"cx\":"<<c.cx()<<",\"cy\":"<<c.cy()<<",\"local_transform\":"<<escaped(c.localTransform().prettyPrint())<<"}\n";
        prior.emplace(id,data);cv::imwrite((out/"prior-images"/(std::to_string(id)+".png")).string(),data.imageRaw());}
    db->closeConnection(false);std::ofstream output(out/"pairs.jsonl");output<<std::setprecision(10);
    auto pair=[&](const std::string &kind,int index,int id,const SensorData &from,const SensorData &to){
        cv::setRNGSeed(0);std::srand(0);RegistrationInfo info;std::unique_ptr<Registration> registration(Registration::create(parameters));
        Transform t=registration->computeTransformation(from,to,Transform(),&info);
        output<<"{\"kind\":"<<escaped(kind)<<",\"index\":"<<index<<",\"prior_id\":"<<id
              <<",\"transform_api_frame\":\"raw Registration::computeTransformation(prior SensorData, query SensorData); both CameraModel localTransform=opticalRotation\""
              <<",\"accepted\":"<<(t.isNull()?"false":"true")
              <<",\"inliers\":"<<info.inliers<<",\"matches\":"<<info.matches<<",\"rejected_reason\":"<<escaped(info.rejectedMsg)<<",\"transform_matrix\":";
        if(t.isNull())output<<"null";else matrix(output,t);output<<"}\n";output.flush();return !t.isNull();};
    int firstId=*ids.begin(),secondId=*std::next(ids.begin());
    bool self=pair("positive_self",-1,firstId,prior.at(firstId),variant(prior.at(firstId),0,false,100000));
    bool neighbor=pair("positive_neighbor",-1,firstId,prior.at(firstId),prior.at(secondId));
    bool black=pair("negative_black",-1,firstId,prior.at(firstId),variant(prior.at(firstId),1,false,100001));
    bool noDepth=pair("negative_no_depth_both",-1,firstId,variant(prior.at(firstId),0,true,firstId),variant(prior.at(firstId),0,true,100002));
    if(!self||!neighbor||black||noDepth){std::cerr<<"pair instrument calibration failed\n";return 3;}
    std::ifstream selection(argv[2]);if(!selection)throw std::runtime_error("missing selection");std::string line;
    while(std::getline(selection,line)){std::istringstream fields(line);std::string category,rgbPath,depthPath;int index;
        if(!(fields>>category>>index>>rgbPath>>depthPath))throw std::runtime_error("invalid selection");
        cv::Mat rgb=cv::imread(rgbPath,cv::IMREAD_COLOR),depth16=cv::imread(depthPath,cv::IMREAD_UNCHANGED),depth;
        if(rgb.size()!=cv::Size(640,480)||depth16.type()!=CV_16UC1||depth16.size()!=rgb.size())throw std::runtime_error("invalid query image");
        depth16.convertTo(depth,CV_32FC1,1./5000.);CameraModel model(517.3,516.5,318.6,255.3,CameraModel::opticalRotation(),0.,rgb.size());
        SensorData query(rgb,depth,model,100000+index);for(int id:ids)pair(category,index,id,prior.at(id),query);}
    std::cout<<"completed calibrated 24-node raw-image forced-pair diagnostic\n";return 0;
}
