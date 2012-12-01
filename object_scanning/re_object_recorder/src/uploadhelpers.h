#ifndef UPLOAD_H
#define UPLOAD_H

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <re_srvs/SetObject.h>
#include <re_srvs/GetObject.h>
#include <re_srvs/UpdateObjectBinaryFile.h>

class UploadBase {
public:
    UploadBase() {}
    virtual bool waitForService() = 0;
    virtual bool check() = 0;
    virtual bool upload() = 0;
};

template <class T>
class UploadBaseHelper : public UploadBase {
public:
    UploadBaseHelper(const std::string& topic) : UploadBase() {
        client = nh.serviceClient<T>(topic);
        get_object_client = nh.serviceClient<re_srvs::GetObject>("re_comm/get_object");
    }

    virtual bool waitForService() {
        if (!client.exists()) {
            std::cout << "waiting 5s for re_comm service to start up..." << std::endl;
            client.waitForExistence(ros::Duration(5.0));
        }
        if (!client.exists()) {
            std::cerr << "re_comm service not found" << std::endl;
            return false;
        }
        return true;
    }

    virtual bool upload() {
        bool servicecall = client.call(request, response);
        if (!servicecall) {
            std::cerr << "upload service call failed" << std::endl;
            return false;
        }
        return response.success;
    }

    bool checkObjectExistence(std::string uid, bool& ok) {
        re_srvs::GetObjectRequest req;
        re_srvs::GetObjectResponse resp;
        req.objectUID = uid;
        ok = get_object_client.call(req, resp);

        return resp.success;
    }

protected:
    ros::NodeHandle nh;
    ros::ServiceClient client, get_object_client;

    typedef typename T::Request Request;
    typedef typename T::Response Response;
    Request request;
    Response response;
};

class UploadSetNewObject : public UploadBaseHelper<re_srvs::SetObject> {
public:
    UploadSetNewObject() : UploadBaseHelper<re_srvs::SetObject>("re_comm/set_object") {
    }

    void fillRequest(std::string apikey, std::string name, std::string clss, std::string owldesc, std::string description, const std::vector<re_msgs::File>& files) {
        request.id = name;
        request.apiKey = apikey;
        request.cls = clss;
        request.description = description;
        request.object = owldesc;
        request.files = files;
    }

    bool check() {
        bool checkOK = false;
        bool exists = checkObjectExistence(request.cls + "." + request.id, checkOK);
        if (exists) {
            std::cerr << "ERROR: object is already in DB" << std::endl;
            return false;
        }
        if (!checkOK) {
            std::cerr << "WARNING: could not check whether object is already in DB" << std::endl;
        }
        return true;
    }
};

class UpdateBinary : public UploadBaseHelper<re_srvs::UpdateObjectBinaryFile> {
public:
    UpdateBinary() : UploadBaseHelper<re_srvs::UpdateObjectBinaryFile>("re_comm/update_object_binary_file") {
    }

    void fillRequest(std::string apikey, std::string objectUID, const re_msgs::File& fileMsg) {
        request.apiKey = apikey;
        request.file = fileMsg;
        request.objectUID = objectUID;
    }

    bool check() {
        bool checkOK = false;
        bool exists = checkObjectExistence(request.objectUID, checkOK);
        if (!exists) {
            std::cerr << "ERROR: object is not in DB" << std::endl;
            return false;
        }
        if (!checkOK) {
            std::cerr << "WARNING: could not check whether object is already in DB" << std::endl;
        }
        return true;
    }
};

#endif // UPLOAD_H
