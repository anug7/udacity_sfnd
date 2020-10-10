#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>


struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};

template<typename T>
class CircularBuffer{
  public:
    CircularBuffer<T>(int _size):
    size(_size), _list(new std::vector<T>(_size)){
      wd_ptr = -1;
    }
    ~CircularBuffer<T>(){
      if (_list != nullptr){
        delete _list;
      }
    }

    void insert(T data){
      wd_ptr = (wd_ptr + 1) % size; 
      _list[++wd_ptr] = data;
    }

    T getElement(int idx){
      int _idx = (wd_ptr - idx) < 0 ? (wd_ptr - idx + size): wd_ptr - idx;
      return _list[_idx % size];
    }
  private:
    std::vector<T> *_list = nullptr;
    int rd_ptr;
    int wd_ptr;
    int size;
};

#endif /* dataStructures_h */
