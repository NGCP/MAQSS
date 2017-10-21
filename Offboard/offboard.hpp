#ifndef OFFBOARD_HPP
#define OFFBOARD_HPP

#include <mutex>

//Allow multiple readers
//Unique writers
class PNav_to_CV {
    public:
        PNav_to_CV();
        bool CV_start() const;
        bool CV_exit() const;
        int get_role() const;
        void set_CV_start(bool set);
        void set_CV_exit(bool set);
        void set_role(int set);
    private:
        mutable std::mutex mutex_;
        bool CV_start_;
        bool CV_exit_;
        int role; //0 -> QuickSearch 1 -> Detailed
};

class CV_to_PNav {
    public:
        CV_to_PNav();
        bool CV_found() const;
        void set_CV_found(bool set);
        void CV_lock();
        void CV_unlock();
    private:
        mutable std::mutex mutex_;
        bool CV_found_;
};

extern PNav_to_CV PeeToCee;
extern CV_to_PNav CeeToPee;
#endif
