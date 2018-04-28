#ifndef OFFBOARD_HPP
#define OFFBOARD_HPP

#include <mutex>
#include <vector>

//Allow multiple readers
//Unique writers
class PNav_to_CV {
    public:
        PNav_to_CV();
        bool CV_start() const;
        bool CV_exit() const;
        int get_role() const;
        int get_lat() const;
        int get_lon() const;
        int get_height() const;
        float get_pitch() const;
        float get_roll() const;
        int get_yaw() const;
        void set_CV_start(bool set);
        void set_CV_exit(bool set);
        void set_role(int set);
        void set_GPS(int lat, int lon, int height, int yaw, float pitchDeg, float rollDeg);
    private:
        mutable std::mutex mutex_;
        bool CV_start_;
        bool CV_exit_;
        int role; //0 -> QuickSearch 1 -> Detailed
        int lat; //Multiply by 1E-7
        int lon; //Multiply by 1E-7
        int height; //Multiply by 1E-3
        int yaw; //Multiply by 1E-2
        float pitch;
        float roll;
};

class CV_to_PNav {
    public:
        CV_to_PNav();
        bool CV_found() const;
        int get_ball_lat() const;
        int get_ball_lon() const;
        void set_CV_found(bool set);
        void set_ball_lat(int lat);
        void set_ball_lon(int lon);
        void CV_lock();
        void CV_unlock();
    private:
        mutable std::mutex mutex_;
        bool CV_found_;
        int latitude_;
        int longitude_;
};

extern PNav_to_CV PeeToCee;
extern CV_to_PNav CeeToPee;
#endif
