#include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/image.hpp"
// #include "sensor_msgs/msg/camera_info.hpp"
// #include "sensor_msgs/image_encodings.hpp"
// #include "image_transport/image_transport.h"

#include "vehicle_interfaces/msg/image.hpp"

#include "vehicle_interfaces/vehicle_interfaces.h"

#include <opencv2/opencv.hpp>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>
#include <vector>

using namespace std::placeholders;
using namespace std::chrono_literals;

void SpinNodeExecutor(rclcpp::executors::SingleThreadedExecutor* exec, std::string threadName);


class Params : public vehicle_interfaces::GenericParams
{
public:
    // Zed RGB & Depth Image Recv
    std::string topic_ZEDCam_RGB_nodeName = "grounddetect_rgb_0_node";
    std::string topic_ZEDCam_RGB_topicName = "zed_rgb_0";
    std::string topic_ZEDCam_Depth_nodeName = "grounddetect_depth_0_node";
    std::string topic_ZEDCam_Depth_topicName = "zed_depth_0";

    // Publish Result
    std::string topic_GroundDetect_nodeName = "grounddetect_img_0_node";
    std::string topic_GroundDetect_topicName = "grounddetect_0";

    int mainCameraWidth = 1280;
    int mainCameraHeight = 720;

    int safetyDirection = 0;// See EmergencyScoreDirection under safety.h
    float safetyDistance = 1500.0;

private:
    void _getParams()
    {
        this->get_parameter("topic_ZEDCam_RGB_nodeName", this->topic_ZEDCam_RGB_nodeName);
        this->get_parameter("topic_ZEDCam_RGB_topicName", this->topic_ZEDCam_RGB_topicName);
        this->get_parameter("topic_ZEDCam_Depth_nodeName", this->topic_ZEDCam_Depth_nodeName);
        this->get_parameter("topic_ZEDCam_Depth_topicName", this->topic_ZEDCam_Depth_topicName);

        this->get_parameter("topic_GroundDetect_nodeName", this->topic_GroundDetect_nodeName);
        this->get_parameter("topic_GroundDetect_topicName", this->topic_GroundDetect_topicName);

        this->get_parameter("mainCameraWidth", this->mainCameraWidth);
        this->get_parameter("mainCameraHeight", this->mainCameraHeight);

        this->get_parameter("safetyDirection", this->safetyDirection);
        this->get_parameter("safetyDistance", this->safetyDistance);
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName)
    {
        this->declare_parameter<std::string>("topic_ZEDCam_RGB_nodeName", this->topic_ZEDCam_RGB_nodeName);
        this->declare_parameter<std::string>("topic_ZEDCam_RGB_topicName", this->topic_ZEDCam_RGB_topicName);
        this->declare_parameter<std::string>("topic_ZEDCam_Depth_nodeName", this->topic_ZEDCam_Depth_nodeName);
        this->declare_parameter<std::string>("topic_ZEDCam_Depth_topicName", this->topic_ZEDCam_Depth_topicName);

        this->declare_parameter<std::string>("topic_GroundDetect_nodeName", this->topic_GroundDetect_nodeName);
        this->declare_parameter<std::string>("topic_GroundDetect_topicName", this->topic_GroundDetect_topicName);

        this->declare_parameter<int>("mainCameraWidth", this->mainCameraWidth);
        this->declare_parameter<int>("mainCameraHeight", this->mainCameraHeight);

        this->declare_parameter<int>("safetyDirection", this->safetyDirection);
        this->declare_parameter<float>("safetyDistance", this->safetyDistance);
        this->_getParams();
    }
};


class ImageSubNode : public vehicle_interfaces::QoSUpdateNode
{
private:
    rclcpp::Subscription<vehicle_interfaces::msg::Image>::SharedPtr subscription_;
    std::string outputDir_;

    cv::Mat recvMat_;
    bool newMatF_;
    std::mutex recvMatLock_;

    cv::Mat outMat_;
    std::mutex outMatLock_;
    std::string topicName_;

private:
    void _topicCallback(const vehicle_interfaces::msg::Image::SharedPtr msg)// JPEG Only
    {
        std::unique_lock<std::mutex> recvMatLocker(this->recvMatLock_, std::defer_lock);
        std::unique_lock<std::mutex> outMatLocker(this->outMatLock_, std::defer_lock);
        recvMatLocker.lock();
        try
        {
            if (msg->format_type == vehicle_interfaces::msg::Image::FORMAT_JPEG)
            {
            	this->recvMat_ = cv::imdecode(msg->data, 1);
            	cv::cvtColor(this->recvMat_, this->recvMat_, cv::COLOR_BGRA2BGR);
            }
            else if (msg->format_type == vehicle_interfaces::msg::Image::FORMAT_RAW && msg->cvmat_type == CV_32FC1)
            {
                float *depths = reinterpret_cast<float *>(&msg->data[0]);
                this->recvMat_ = cv::Mat(msg->height, msg->width, CV_32FC1, depths);
            }
            else
                throw -1;
            this->newMatF_ = true;
            outMatLocker.lock();
            this->outMat_ = this->recvMat_.clone();
            outMatLocker.unlock();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        catch(...)
        {
            std::cerr << "[ImageSubNode::_topicCallback] Unknown Exception.\n";
        }
        recvMatLocker.unlock();
#ifdef NODE_SUBSCRIBE_PRINT
        RCLCPP_INFO(this->get_logger(), "I heard: foramt: %d, width: %d, height: %d", 
            msg->format_order_type, msg->width, msg->height);
#endif
    }

    void _qosCallback(std::map<std::string, rclcpp::QoS*> qmap)
    { 
        for (const auto& [k, v] : qmap)
        {
            if (k == this->topicName_ || k == (std::string)this->get_namespace() + "/" + this->topicName_)
            {
                this->subscription_.reset();
                this->subscription_ = this->create_subscription<vehicle_interfaces::msg::Image>(this->topicName_, 
                    *v, std::bind(&ImageSubNode::_topicCallback, this, std::placeholders::_1));
            }
        }
    }

public:
    ImageSubNode(const std::string& nodeName, 
                    const std::string& topicName, 
                    cv::Mat& initMat, 
                    const std::string& qosServiceName, 
                    const std::string& qosDirPath) : 
        vehicle_interfaces::QoSUpdateNode(nodeName, qosServiceName, qosDirPath), 
        rclcpp::Node(nodeName), 
            topicName_(topicName)
    {
        this->recvMat_ = initMat.clone();
        this->outMat_ = initMat.clone();
        this->newMatF_ = false;

        this->addQoSCallbackFunc(std::bind(&ImageSubNode::_qosCallback, this, std::placeholders::_1));
        vehicle_interfaces::QoSPair qpair = this->addQoSTracking(topicName);
        this->subscription_ = this->create_subscription<vehicle_interfaces::msg::Image>(topicName, 
            *qpair.second, std::bind(&ImageSubNode::_topicCallback, this, std::placeholders::_1));
    }

    bool getImage(cv::Mat& outputMat, bool& newF)
    {
        newF = this->newMatF_;
        std::unique_lock<std::mutex> outMatLocker(this->outMatLock_, std::defer_lock);
        outMatLocker.lock();
        outputMat = this->outMat_.clone();
        outMatLocker.unlock();
        this->newMatF_ = false;
    }
};


class GroundDetectPublisher : public vehicle_interfaces::TimeSyncNode, public vehicle_interfaces::QoSUpdateNode
{
private:
    std::shared_ptr<Params> params_;
    rclcpp::Publisher<vehicle_interfaces::msg::Image>::SharedPtr pub_;
    std::mutex pubLock_;

private:
    void _qosCallback(std::map<std::string, rclcpp::QoS*> qmap)
    {
        std::unique_lock<std::mutex> locker(this->pubLock_, std::defer_lock);
        for (const auto& [k, v] : qmap)
        {
            if (k == this->params_->topic_GroundDetect_nodeName || k == (std::string)this->get_namespace() + "/" + this->params_->topic_GroundDetect_nodeName)
            {
                locker.lock();
                this->pub_.reset();// Call destructor
                this->pub_ = this->create_publisher<vehicle_interfaces::msg::Image>(this->params_->topic_GroundDetect_topicName, *v);
                locker.unlock();
            }
        }
    }

public:
    GroundDetectPublisher(const std::shared_ptr<Params>& params) : 
        vehicle_interfaces::TimeSyncNode(params->topic_GroundDetect_nodeName, params->timesyncService, params->timesyncPeriod_ms, params->timesyncAccuracy_ms, params->timesyncWaitService), 
        vehicle_interfaces::QoSUpdateNode(params->topic_GroundDetect_nodeName, params->qosService, params->qosDirPath), 
        rclcpp::Node(params->topic_GroundDetect_nodeName), 
        params_(params)
    {
        this->addQoSCallbackFunc(std::bind(&GroundDetectPublisher::_qosCallback, this, std::placeholders::_1));
        vehicle_interfaces::QoSPair qpair = this->addQoSTracking(params->topic_GroundDetect_topicName);
        if (qpair.first == "")
            RCLCPP_ERROR(this->get_logger(), "[GroundDetectPublisher] Failed to add topic to track list: %s", params->topic_GroundDetect_topicName.c_str());
        else
        {
            RCLCPP_INFO(this->get_logger(), "[GroundDetectPublisher] QoS profile [%s]:\nDepth: %d\nReliability: %d", 
                qpair.first.c_str(), qpair.second->get_rmw_qos_profile().depth, qpair.second->get_rmw_qos_profile().reliability);
        }
        this->pub_ = this->create_publisher<vehicle_interfaces::msg::Image>(params->topic_GroundDetect_topicName, *qpair.second);
    }

    void pubImage(const std::vector<uchar>& dataVec, const cv::Size& sz)
    {
        static u_int64_t frame_id = 0;
        auto msg = vehicle_interfaces::msg::Image();
        msg.header.priority = vehicle_interfaces::msg::Header::PRIORITY_SENSOR;
        msg.header.device_type = vehicle_interfaces::msg::Header::DEVTYPE_IMAGE;
        msg.header.device_id = this->params_->topic_GroundDetect_nodeName;
        msg.header.frame_id = frame_id++;
        msg.header.stamp_type = this->getTimestampType();
        msg.header.stamp = this->getTimestamp();
        msg.header.stamp_offset = this->getCorrectDuration().nanoseconds();
        msg.header.ref_publish_time_ms = 0;// Depend on ZED publisher

        msg.format_type = msg.FORMAT_JPEG;
        msg.width = sz.width;
        msg.height = sz.height;
        msg.data = dataVec;

        std::unique_lock<std::mutex> locker(this->pubLock_, std::defer_lock);
        locker.lock();
        this->pub_->publish(msg);
        locker.unlock();
    }
};


class ZEDNode : public vehicle_interfaces::VehicleServiceNode
{
private:
    std::shared_ptr<Params> params_;
    std::shared_ptr<ImageSubNode> rgbNode_;
    std::shared_ptr<ImageSubNode> depthNode_;

    rclcpp::executors::SingleThreadedExecutor* rgbExec_;
    rclcpp::executors::SingleThreadedExecutor* depthExec_;

    std::thread rgbNodeTh_;
    std::thread depthNodeTh_;

    cv::Mat rgbInitMat_;
    cv::Mat depthInitMat_;

    bool exitF_;

public:
    ZEDNode(const std::shared_ptr<Params>& params) : 
        VehicleServiceNode(params), 
        rclcpp::Node(params->nodeName), 
        exitF_(false)
    {
        this->params_ = params;
        this->rgbInitMat_ = cv::Mat(params->mainCameraHeight, params->mainCameraWidth, CV_8UC3, cv::Scalar(50));
        this->depthInitMat_ = cv::Mat(params->mainCameraHeight, params->mainCameraWidth, CV_32FC1, cv::Scalar(50));
        
        this->rgbNode_ = std::make_shared<ImageSubNode>(params->topic_ZEDCam_RGB_nodeName, 
                                                        params->topic_ZEDCam_RGB_topicName, 
                                                        this->rgbInitMat_, 
                                                        params->qosService, 
                                                        params->qosDirPath);
        this->rgbExec_ = new rclcpp::executors::SingleThreadedExecutor();
        this->rgbExec_->add_node(this->rgbNode_);
        this->rgbNodeTh_ = std::thread(SpinNodeExecutor, this->rgbExec_, params->topic_ZEDCam_RGB_nodeName);
        
        this->depthNode_ = std::make_shared<ImageSubNode>(params->topic_ZEDCam_Depth_nodeName, 
                                                            params->topic_ZEDCam_Depth_topicName, 
                                                            this->depthInitMat_, 
                                                            params->qosService, 
                                                            params->qosDirPath);
        this->depthExec_ = new rclcpp::executors::SingleThreadedExecutor();
        this->depthExec_->add_node(this->depthNode_);
        this->depthNodeTh_ = std::thread(SpinNodeExecutor, this->depthExec_, params->topic_ZEDCam_Depth_nodeName);
    }

    ~ZEDNode()
    {
        close();
    }

    void getRGBImage(cv::Mat& outputMat, bool& newF)
    {
        this->rgbNode_->getImage(outputMat, newF);
    }

    void getDepthImage(cv::Mat& outputMat, bool& newF)
    {
        this->depthNode_->getImage(outputMat, newF);
    }

    void close()
    {
        if (this->exitF_)
            return;
        this->rgbExec_->cancel();
        this->depthExec_->cancel();
        this->rgbNodeTh_.join();
        this->depthNodeTh_.join();
        this->exitF_ = true;
    }
};


class WorkingRate
{
private:
    float rate_;
    int frameCnt_;
    std::chrono::duration<int, std::milli> interval_ms_;
    vehicle_interfaces::Timer* timer_;

    std::mutex locker_;

private:
    template <typename T>
    void _safeSave(T* ptr, const T value, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        *ptr = value;
    }

    template <typename T>
    T _safeCall(const T* ptr, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        return *ptr;
    }

    void _timerCallback()
    {
        int cnt = this->_safeCall(&this->frameCnt_, this->locker_);
        std::chrono::duration<float> toSec = this->interval_ms_;// Casting msec to sec
        this->_safeSave(&this->rate_, (float)cnt / toSec.count(), this->locker_);
        this->_safeSave(&this->frameCnt_, 0, this->locker_);
    }

public:
    WorkingRate(int interval_ms)
    {
        this->rate_ = 0;
        this->frameCnt_ = 0;
        this->interval_ms_ = std::chrono::milliseconds(interval_ms);
        this->timer_ = new vehicle_interfaces::Timer(interval_ms, std::bind(&WorkingRate::_timerCallback, this));
    }

    ~WorkingRate()
    {
        this->timer_->destroy();
        delete this->timer_;
    }

    void addCnt(int num) { this->_safeSave(&this->frameCnt_, this->frameCnt_ + num, this->locker_); }

    void addOneCnt() { this->_safeSave(&this->frameCnt_, this->frameCnt_ + 1, this->locker_); }

    void start() { this->timer_->start(); }

    void stop() { this->timer_->stop(); }

    float getRate() { return this->_safeCall(&this->rate_, this->locker_); }
};


class MultiWorkingRate// std::vector or template T
{
private:
    std::vector<float> rateVec_;
    std::vector<int> cntVec_;
    std::chrono::duration<int, std::milli> interval_ms_;
    vehicle_interfaces::Timer* timer_;

    std::mutex locker_;

private:
    void _timerCallback()
    {
        std::lock_guard<std::mutex> _lock(this->locker_);
        std::chrono::duration<float> toSec = this->interval_ms_;// Casting msec to sec
        for (int i = 0; i < this->cntVec_.size(); i++)
        {
            this->rateVec_[i] = (float)this->cntVec_[i] / toSec.count();
            this->cntVec_[i] = 0;
        }
    }

public:
    MultiWorkingRate(int interval_ms, int numOfWorkers)
    {
        this->rateVec_ = std::vector<float>(numOfWorkers, 0);
        this->cntVec_ = std::vector<int>(numOfWorkers, 0);
        this->interval_ms_ = std::chrono::milliseconds(interval_ms);
        this->timer_ = new vehicle_interfaces::Timer(interval_ms, std::bind(&MultiWorkingRate::_timerCallback, this));
    }

    ~MultiWorkingRate()
    {
        this->timer_->destroy();
        delete this->timer_;
    }

    void addOneCnt(int idx = -1)// idx < 0: all indices add one
    {
        std::lock_guard<std::mutex> _lock(this->locker_);
        if (idx < 0)
            for (auto& i : this->cntVec_)
                i++;
        else
            this->cntVec_[idx]++;
    }

    void addCnt(std::vector<int> numVec)// Same size to initial size
    {
        std::lock_guard<std::mutex> _lock(this->locker_);
        for (int i = 0; i < this->cntVec_.size(); i++)
            this->cntVec_[i] += numVec[i];
    }

    void start() { this->timer_->start(); }

    void stop() { this->timer_->stop(); }

    std::vector<float> getRate()
    {
        std::lock_guard<std::mutex> _lock(this->locker_);
        return this->rateVec_;
    }
};


/**
 * Functions
 */
void SpinNodeExecutor(rclcpp::executors::SingleThreadedExecutor* exec, std::string threadName)
{
    std::this_thread::sleep_for(1s);
    std::cerr << threadName << " start..." << std::endl;
    exec->spin();
    std::cerr << threadName << " exit." << std::endl;
}

void SpinNode(std::shared_ptr<rclcpp::Node> sub, std::string threadName)
{
    std::cerr << threadName << " start..." << std::endl;
    rclcpp::spin(sub);
    std::cerr << threadName << " exit." << std::endl;
    rclcpp::shutdown();
}
