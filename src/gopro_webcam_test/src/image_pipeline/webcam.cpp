#include "image_pipeline/webcam.hpp"

#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
using namespace std::chrono_literals;
namespace gopro {

// ----- Logging Helper Implementations -----
void log_debug(const std::string & msg) {
    std::cout << "[DEBUG] " << msg << std::endl;
}

void log_info(const std::string & msg) {
    std::cout << "[INFO] " << msg << std::endl;
}

void log_error(const std::string & msg) {
    std::cerr << "[ERROR] " << msg << std::endl;
}

// ----- HTTP Helper Functions -----
static size_t WriteCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    size_t totalSize = size * nmemb;
    std::string* str = static_cast<std::string*>(userp);
    str->append(static_cast<char*>(contents), totalSize);
    return totalSize;
}

HttpResponse http_get(const std::string & url) {
    CURL *curl = curl_easy_init();
    if (!curl) {
        throw std::runtime_error("Could not initialize curl");
    }
    std::string response_string;
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
       std::string err = curl_easy_strerror(res);
       curl_easy_cleanup(curl);
       throw std::runtime_error("Curl error: " + err);
    }
    long http_code = 0;
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);
    curl_easy_cleanup(curl);
    return {http_code, response_string};
}

std::string build_url(const std::string & base, const std::map<std::string, std::string>& params) {
    if (params.empty()) return base;
    std::ostringstream oss;
    oss << base << "?";
    bool first = true;
    for (const auto & kv : params) {
        if (!first) {
            oss << "&";
        }
        oss << kv.first << "=" << kv.second;
        first = false;
    }
    return oss.str();
}

// ----- Webcam Implementation -----
Webcam::Webcam(const std::string & serial) : state(State::DISABLED) {
    if (serial.size() < 3) {
        throw std::runtime_error("Serial must have at least 3 characters");
    }
    // Format IP as "172.2{d1}.1{d2}{d3}.51"
    char d1 = serial[serial.size()-3];
    char d2 = serial[serial.size()-2];
    char d3 = serial[serial.size()-1];
    ip = "172.2" + std::string(1, d1) + ".1" + std::string(1, d2) + std::string(1, d3) + ".51";
    base_url = "http://" + ip + ":8080/gopro/";
}

std::string Webcam::endpoint_to_string(Endpoint ep) {
    switch(ep) {
        case Endpoint::WIRELESS_USB_DISABLE:
            return "camera/control/wired_usb?p=0";
        case Endpoint::GET_DATE_TIME:
            return "camera/get_date_time";
        case Endpoint::GET_WEBCAM_STATUS:
            return "webcam/status";
        case Endpoint::START_PREVIEW:
            return "webcam/preview";
        case Endpoint::START_WEBCAM:
            return "webcam/start";
        case Endpoint::STOP_WEBCAM:
            return "webcam/stop";
        case Endpoint::DISABLE_WEBCAM:
            return "webcam/exit";
        default:
            throw std::runtime_error("Invalid endpoint");
    }
}

HttpResponse Webcam::_send_http_no_validate(Endpoint ep, const std::map<std::string, std::string>& params) {
    std::string url = base_url + endpoint_to_string(ep);
    url = build_url(url, params);
    log_debug("Sending GET request to: " + url);
    HttpResponse response = http_get(url);
    log_debug("HTTP return code: " + std::to_string(response.code));
    try {
        auto j = json::parse(response.body);
        log_debug("Response JSON: " + j.dump(4));
    } catch (...) {
        log_debug("Response is not valid JSON");
    }
    return response;
}

HttpResponse Webcam::_send_http(Endpoint ep, const std::map<std::string, std::string>& params) {
    HttpResponse response = _send_http_no_validate(ep, params);
    if (response.code != 200) {
        throw std::runtime_error("HTTP request failed with code " + std::to_string(response.code));
    }
    return response;
}

void Webcam::enable() {
    _send_http_no_validate(Endpoint::WIRELESS_USB_DISABLE);
    state = State::READY;
}

void Webcam::preview() {
    log_info("Starting preview");
    _send_http(Endpoint::START_PREVIEW);
    state = State::LOW_POWER_PREVIEW;
}

void Webcam::start(int port, int resolution, int fov) {
    log_info("Starting webcam");
    std::map<std::string, std::string> params;
    if (port != -1) {
        params["port"] = std::to_string(port);
    }
    if (resolution != -1) {
        params["res"] = std::to_string(resolution);
    }
    if (fov != -1) {
        params["fov"] = std::to_string(fov);
    }
    _send_http(Endpoint::START_WEBCAM, params);
    state = State::HIGH_POWER_PREVIEW;
    log_info("Webcam started successfully");
}

void Webcam::stop() {
    log_info("Stopping webcam");
    _send_http(Endpoint::STOP_WEBCAM);
    state = State::READY;
}

void Webcam::disable() {
    log_info("Disabling webcam");
    _send_http(Endpoint::DISABLE_WEBCAM);
    state = State::DISABLED;
}

// ----- Player Implementation -----
Player::Player() : stop_player_(false),player_started_(false) {}

bool Player::is_running() const {
    return player_started_;
}
std::string Player::get_url() const {
    return url_;
}

void Player::set_url(const std::string & u) {
    if (is_running()) {
        throw std::runtime_error("Cannot set URL while player is running");
    }
    url_ = u;
}

void Player::start(const std::string & stream_url) {
    log_info("Starting player @ " + stream_url);
    set_url(stream_url);
    player_thread_ = std::thread(&Player::_run, this);
    // Wait until the player thread has started.
    while (!player_started_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void Player::stop() {
    if (is_running()) {
        log_info("Stopping player");
        stop_player_ = true;
        if (player_thread_.joinable())
            player_thread_.join();
    }
}

void Player::_run() {
    std::string full_url = url_ + "?overrun_nonfatal=1&fifo_size=50000000&port=7567";
    std::cout << full_url << std::endl;
    cv::VideoCapture cap(full_url, cv::CAP_FFMPEG);
    if (!cap.isOpened()) {
        log_error("Failed to open video stream at " + full_url);
        return;
    }
    player_started_ = true;
    log_info("Player started.");
    while (!stop_player_) {
        // cv::Mat frame;
        bool ret = cap.read(frame_);
        std::this_thread::sleep_for(3ms);
        // if (ret && !frame.empty()) {
        //     cv::imshow("frame", frame);
        // }
        if (cv::waitKey(1) >= 0) break;
    }
    cap.release();
    cv::destroyAllWindows();
}

// ----- GoProWebcamPlayer Implementation -----
std::set<int> GoProWebcamPlayer::used_ports_;
int GoProWebcamPlayer::free_port_ = 8554;

int GoProWebcamPlayer::get_free_port() {
    while (used_ports_.find(free_port_) != used_ports_.end()) {
        free_port_++;
    }
    return free_port_++;
}

std::string GoProWebcamPlayer::STREAM_URL(int port) {
    return "udp://0.0.0.0:" + std::to_string(port);
}

GoProWebcamPlayer::GoProWebcamPlayer(const std::string & serial, int port)
    : webcam_(serial), player_() {
    if (port != -1) {
        if (used_ports_.find(port) != used_ports_.end()) {
            throw std::runtime_error("Port " + std::to_string(port) + " is already being used");
        }
        this->port_ = port;
    } else {
        this->port_ = get_free_port();
    }
    used_ports_.insert(this->port_);
    log_debug("Using port " + std::to_string(this->port_));
}

void GoProWebcamPlayer::open() {
    webcam_.enable();
}

int GoProWebcamPlayer::get_port() const {
    return port_;
}

void GoProWebcamPlayer::play(int resolution, int fov) {
    webcam_.start(port_, resolution, fov);
    player_.start(STREAM_URL(port_));
}

void GoProWebcamPlayer::close() {
    player_.stop();
    webcam_.disable();
}

// ----- GoProStream Implementation -----
GoProStream::GoProStream(const std::vector<int>& serial_number, int port)
  : port_(port)
{
  // Convert the vector of ints into a string (e.g., {5,3,7} becomes "537")
  std::ostringstream oss;
  for (int num : serial_number) {
    oss << num;
  }
  std::string serial_str = oss.str();

  // Create the GoProWebcamPlayer with the serial string and port.
  webcam_player_ = std::make_unique<gopro::GoProWebcamPlayer>(serial_str, port_);

  // Start the video stream.
  start_stream();

  // Build the stream URL.
  // We use the STREAM_URL() static method from GoProWebcamPlayer.
  std::string stream_url = gopro::GoProWebcamPlayer::STREAM_URL(port_) +
                           "?overrun_nonfatal=1&fifo_size=50000000&fov=4&port=7567&res=7";
  std::cout<< stream_url<<std::endl;
  // Open the video stream with OpenCV (using FFMPEG).
  if (!cap_.open(stream_url, cv::CAP_FFMPEG)) {
    std::cerr << "Error: Unable to open video stream at " << stream_url << std::endl;
  }
}

void GoProStream::start_stream() {
  // Configure the GoPro by enabling the webcam and starting the stream.
  webcam_player_->open();
  webcam_player_->play();
}

std::pair<bool, cv::Mat> GoProStream::image_capture() {
  cv::Mat frame;
  bool ret = image_capture(frame);
  // (Optionally, you might process frame here, e.g. undistort_image(frame).)
  return std::make_pair(ret, frame);
}
bool GoProStream::image_capture(cv::Mat& frame) {
    bool ret = cap_.read(frame);
    // (Optionally, you might process frame here, e.g. undistort_image(frame).)
    return ret;
}

} // namespace gopro

// // ----- Main (Example Usage) -----
// int main(int argc, char** argv) {

//     (void)argc;
//     (void)argv;
//     try {
//         std::string serial = "123"; // example serial number (at least three characters)
//         gopro::GoProWebcamPlayer player_(serial);
//         player_.open();
//         player_.play();
//         std::this_thread::sleep_for(std::chrono::seconds(10));
//         player_.close();
//     } catch (const std::exception & e) {
//         std::cerr << "Exception: " << e.what() << std::endl;
//     }
//     return 0;
// }
