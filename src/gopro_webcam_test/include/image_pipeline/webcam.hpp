#ifndef WEBCAM_HPP
#define WEBCAM_HPP

#include <string>
#include <map>
#include <set>
#include <stdexcept>
#include <thread>
#include <atomic>

// Third-party libraries:
#include <curl/curl.h>               // For HTTP requests (libcurl)
#include <nlohmann/json.hpp>         // For JSON parsing (header-only library)
#include <opencv2/opencv.hpp>        // For video capture and display

// Use the nlohmann::json namespace alias for convenience.
using json = nlohmann::json;

namespace gopro {

// ----- Logging Helpers -----
void log_debug(const std::string & msg);
void log_info(const std::string & msg);
void log_error(const std::string & msg);

// ----- HTTP Helper Types and Functions -----
struct HttpResponse {
    long code;
    std::string body;
};

HttpResponse http_get(const std::string & url);
std::string build_url(const std::string & base, const std::map<std::string, std::string>& params);

// ----- Webcam Class -----
// This class configures the GoPro as a webcam via HTTP.
class Webcam {
public:
    // Endpoints used for HTTP requests.
    enum class Endpoint {
        WIRELESS_USB_DISABLE,
        GET_DATE_TIME,
        GET_WEBCAM_STATUS,
        START_PREVIEW,
        START_WEBCAM,
        STOP_WEBCAM,
        DISABLE_WEBCAM
    };

    // State of the webcam.
    enum class State {
        DISABLED,
        READY,
        LOW_POWER_PREVIEW,
        HIGH_POWER_PREVIEW
    };

    // Constructor requires a serial string (at least three characters).
    Webcam(const std::string & serial);

    // Control functions.
    void enable();
    void preview();
    // Optional parameters: pass -1 if a parameter is not set.
    void start(int port = -1, int resolution = -1, int fov = -1);
    void stop();
    void disable();

private:
    std::string ip;
    std::string base_url;
    State state;

    // Converts an endpoint enum into its URL string.
    std::string endpoint_to_string(Endpoint ep);

    // Sends HTTP GET requests.
    HttpResponse _send_http_no_validate(Endpoint ep, const std::map<std::string, std::string>& params = {});
    HttpResponse _send_http(Endpoint ep, const std::map<std::string, std::string>& params = {});
};

// ----- Player Class -----
// This class opens a video stream via OpenCV in a separate thread.
class Player {
public:
    Player();
    bool is_running() const;
    std::string get_url() const;
    void set_url(const std::string & u);
    void start(const std::string & stream_url);
    void stop();
    cv::Mat get_frame(){return frame_;}

private:
    std::string url_;
    std::thread player_thread_;
    std::atomic<bool> stop_player_;
    std::atomic<bool> player_started_;
    void _run();
    cv::Mat frame_;
};

// ----- GoProWebcamPlayer Class -----
// A high-level class that combines the Webcam and Player to manage a GoPro stream.
class GoProWebcamPlayer {
public:
    // Returns a stream URL given a port.
    static std::string STREAM_URL(int port);

    // Constructor takes a serial (at least three characters) and an optional port.
    GoProWebcamPlayer(const std::string & serial, int port = -1);

    // Configures the GoPro.
    void open();
    // Returns the port in use.
    int get_port() const;
    // Starts the webcam stream (optionally specifying resolution and FOV).
    void play(int resolution = 12, int fov = 4);//1080 linear (12,4), 720 linear (7,4)
    // Stops the stream and disables the webcam.
    void close();
    cv::Mat get_frame(){return player_.get_frame();}

private:
    Webcam webcam_;
    Player player_;
    int port_;
    static std::set<int> used_ports_;
    static int free_port_;
    static int get_free_port();
};
    
class GoProStream {
public:
    // Constructor: serial_number is provided as a vector of ints, port is the HTTP port.
    GoProStream(const std::vector<int>& serial_number = {5, 3, 7}, int port = 7567);

    // Starts the video stream.
    void start_stream();

    // Captures one image frame. Returns a pair: {success, frame}.
    std::pair<bool, cv::Mat> image_capture();
    bool image_capture(cv::Mat& frame);
    cv::Mat get_frame(){return webcam_player_->get_frame();}

private:
    int port_;
    // Use a unique_ptr for managing the GoProWebcamPlayer instance.
    std::unique_ptr<gopro::GoProWebcamPlayer> webcam_player_;
    cv::VideoCapture cap_;
};
} // namespace gopro

#endif // WEBCAM_HPP
