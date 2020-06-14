#ifndef MAT_READER_H
#define MAT_READER_H

#include <ros/ros.h>
#include <matio.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <iostream>

namespace fastrack_local_planner
{

    class MATReader
    {
    public:
        ~MATReader() { Close(); }
        MATReader()
            : mat_fp_(nullptr) {}
        MATReader(const std::string &file_name)
            : mat_fp_(nullptr)
        {
            if (!Open(file_name))
            {
                ROS_ERROR("MATReader: could not load file %s.",
                          file_name.c_str());
            }
        }

        // Open and close a file. Open returns bool upon success.
        bool Open(const std::string &file_name);
        void Close()
        {
            if (IsOpen())
                Mat_Close(mat_fp_);
        }

        // Is this reader open?
        bool IsOpen() { return mat_fp_; }

        // Read string. Returns bool indicating success.
        bool ReadString(const std::string &field_name, std::string *value);

        // Read scalar. Returns bool indicating success.
        bool ReadScalar(const std::string &field_name, double *value);
        bool ReadScalar(const std::string &field_name, size_t *value);

        // Read vector. Returns bool indicating success.
        bool ReadVector(const std::string &field_name, std::vector<double> *values);
        bool ReadVector(const std::string &field_name, std::vector<size_t> *values);

    private:
        mat_t *mat_fp_;
    }; //\class MATReader

} // namespace fastrack_local_planner

#endif