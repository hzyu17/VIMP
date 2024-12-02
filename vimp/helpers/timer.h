/**
 * @file timer.h
 * @author Hongzhe Yu (hyu419@gatech.edu)
 * @brief A C++ timer class to help compute elapsed time.
 * https://stackoverflow.com/questions/2808398/easily-measure-elapsed-time
 * @version 0.1
 * @date 2023-01-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

//***C++11 Style:***
#include <chrono>
#include <iostream>

using namespace std;

class Timer{
    typedef chrono::steady_clock::time_point time_point;
public:
    Timer(){
        _start = chrono::steady_clock::now();
    };
    time_point now(){
        return chrono::steady_clock::now();
    }

    void start(){
        _start = chrono::steady_clock::now();
        _has_started = true;
    }

    double end_sec(const std::string & header="Time difference = "){
        if (!_has_started){
            cout << "Not started yet, must call start() first! " << endl;
            return -999.9;
        }else{
            _end = chrono::steady_clock::now();
            auto time_elapsed_ms = chrono::duration_cast<chrono::milliseconds>(_end - _start).count();
            cout << header << " " << time_elapsed_ms << "[ms]" << endl;
            _has_started = false;

            double time_elapsed_sec = time_elapsed_ms / 1e3;
            return time_elapsed_sec;
        }
        
    }

    double end_mis(const std::string & header="Time difference = "){
        if (!_has_started){
            cout << "Not started yet, must call start() first! " << endl;
            return -999.9;
        }else{
            _end = chrono::steady_clock::now();
            auto time_elapsed_ms = chrono::duration_cast<chrono::milliseconds>(_end - _start).count();
            // cout << header << " " << time_elapsed_ms << "[ms]" << endl;
            _has_started = false;

            return time_elapsed_ms;
        }
        
    }

    void end_mus(const std::string & header="Time difference = "){
        if (!_has_started){
            cout << "Not started yet, must call start() first! " << endl;
        }else{
            _end = chrono::steady_clock::now();
            auto time_elapsed_us = chrono::duration_cast<chrono::microseconds>(_end - _start).count();
            cout << header << " " << time_elapsed_us << "[us]" << endl;
            _has_started = false;
        }
    }

private:
    time_point _start;
    time_point _end;

    bool _has_started;

};
