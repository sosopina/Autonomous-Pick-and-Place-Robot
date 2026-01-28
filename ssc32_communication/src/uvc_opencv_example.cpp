//-------------------------------------
// For X11 : right-click project -> set build Host -> choose and Manage Host -> Host Properties -> enable X11 Forwarding  

// touche F5 pour compiler et debugger
// ctrl shift i pour auto indent

#include <chrono>
#include <iostream>
#include <libuvc/libuvc.h>
#include <opencv2/opencv.hpp>
#include "uvc_opencv_example.hpp"

using namespace std;
using namespace cv;
using namespace chrono;

bool convert_to_bgr(uvc_frame_t *frame, cv::Mat &img) {
    static bool first_conversion = true;
    if (frame->frame_format != UVC_FRAME_FORMAT_MJPEG) {
        cout << "bad format" << endl;
        return false;
    }
    // Créer un cv::Mat à partir du buffer MJPEG
    vector<uchar> buf((uchar *) frame->data, (uchar *) frame->data + frame->data_bytes);
    if (first_conversion) {
        img = cv::imdecode(buf, cv::IMREAD_COLOR); // RGB/BGR
        if (!img.empty()) {
            first_conversion = false;
        }
    } else {
        img = cv::imdecode(buf, cv::IMREAD_COLOR, &img); // no reallocation
    }
    return !img.empty();
}

void my_frame_callback(uvc_frame_t *frame, void *ptr) {
    static auto start = chrono::high_resolution_clock::now();
    // execution context
    struct_opencv_example*so = (struct_opencv_example *) ptr;
    // Displays time in output file in
    // microseconds
    if (so->verbose & VERBOSE_TIME_MSK) {
        auto stop = high_resolution_clock::now();
        // Difference is calculated
        auto duration = duration_cast<microseconds>(stop - start);
        start = stop;
        cout << "Time taken in microseconds : " << duration.count() << endl;
    }
    static int n = 0;
    static uvc_frame_t *bgr = NULL;
    static cv::Mat img;
    // ptr peut pointer vers une structure utilisateur, ici on s'en sert pas
    if (!frame || !frame->data || frame->data_bytes == 0)
        return;
    if (n == 0) {
        // On convertit en B  GR (format OpenCV)
        cout << " allocate mem bgr" << endl;
        bgr = uvc_allocate_frame(frame->width * frame->height * 3);
    }

    if (!bgr)
        return;
    if (so->verbose & VERBOSE_COUNT_MSK) {
        cout << " count =" << n;
    }
    n++;
    if (so->verbose & VERBOSE_CONV_MSK) {
        cout << " conv frame " << frame->width << "x" << frame->height;
    }
    bool ok = convert_to_bgr(frame, img);
    // uvc_error_t conv = uvc_any2bgr(frame, bgr);
    //  uvc_error_t conv=UVC_SUCCESS

    if (!ok) {
        cout << "CONVERSION IMPOSSIBLE, format=" << frame->frame_format << endl;
        uvc_free_frame(bgr);
        bgr = NULL;
        return;
    }
    if (so->verbose & VERBOSE_CONV_MSK) {
        cout << "  ok";
    }
    if (so->mode_xwindows == USE_XWINDOW) {
        cv::imshow("Flux UVC", img);
        cv::waitKey(1); // nécessaire pour rafraîchir la fenêtre
    }
    if (so->verbose != 0) {
        cout << endl;
    }
}


// ============================================================================
// MAIN
// ============================================================================

int vrai_main(struct_opencv_example config) {
    uvc_context_t *ctx = NULL;
    uvc_device_t *dev = NULL;
    uvc_device_handle_t *devh = NULL;
    uvc_stream_ctrl_t ctrl;
    uvc_error_t res;
    // not volatile local conf
    struct_opencv_example *local_conf = (struct_opencv_example *) malloc(sizeof (struct_opencv_example));
    *local_conf = config;
    // 1?? Initialisation du contexte
    res = uvc_init(&ctx, NULL);
    if (res < 0) {
        uvc_perror(res, "uvc_init");
        return -1;
    }
    cout << "libuvc initialisee." << endl;

    // 2?? Trouver le premier périphérique UVC connecté
    res = uvc_find_device(ctx, &dev, 0, 0, NULL); // vendorID, productID, serialNumber

    if (res < 0) {
        uvc_perror(res, "uvc_find_device");
        uvc_exit(ctx);
        return -1;
    }

    // 3?? Ouvrir le périphérique
    res = uvc_open(dev, &devh);
    if (res < 0) {
        uvc_perror(res, "uvc_open");
        uvc_unref_device(dev);
        uvc_exit(ctx);
        return -1;
    }

    cout << "Caméra ouverte." << endl;

    // 4?? Choisir le mode de capture (exemple : MJPEG 640x480 à 30 fps)
    res = uvc_get_stream_ctrl_format_size(
            devh, &ctrl, UVC_FRAME_FORMAT_MJPEG, 1280, 960, 30);
    if (res < 0) {
        uvc_perror(res, "uvc_get_stream_ctrl_format_size");
        uvc_close(devh);
        uvc_unref_device(dev);
        uvc_exit(ctx);
        return -1;
    }

    // 5?? Démarrer le flux avec le callback
    res = uvc_start_streaming(devh, &ctrl, my_frame_callback, local_conf, 0);
    if (res < 0) {
        uvc_perror(res, "uvc_start_streaming");
        uvc_close(devh);
        uvc_unref_device(dev);
        uvc_exit(ctx);
        return -1;
    }

    cout << "Streaming en cours... (ESC pour quitter)" << endl;

    // 6?? Boucle principale (juste pour garder le programme en vie)
    while (true) {
        //    int key = cv::waitKey(10);
        //    if (key == 27) break; // ESC pour quitter
    }

    // 7?? Arrêter le flux et libérer les ressources
    uvc_stop_streaming(devh);
    cout << "Flux arrêté." << endl;

    uvc_close(devh);
    uvc_unref_device(dev);
    uvc_exit(ctx);

    return 0;
}

int main() {
    struct_opencv_example so;

    string choice;
    system("env"); // exécute la commande
    //system("export"); // exécute la commande
    
    cout << "Type your choice ( 1 ; xwindows with vnc server, or 2 : no xwindows ): ";
    cin >> choice; // get user input from the keyboard
    int i = -1;
    try {
        i = std::stoi(choice);
    } catch (const std::invalid_argument& e) {
        std::cout << "Erreur: pas un nombre" << std::endl;
        return 0;
    }
    if (i < 0) {
        std::cout << "choice <=0 : abort" << std::endl;
        return 0;
    }
    so.verbose = VERBOSE_TIME_MSK|VERBOSE_CONV_MSK; //|VERBOSE_COUNT_MSK|VERBOSE_CONV_MSK;
    switch (i) {
        case 1:so.mode_xwindows = USE_XWINDOW;
            break;
        case 2:so.mode_xwindows = NO_XWINDOW;
            break;
        default:so.mode_xwindows = NO_XWINDOW;
    }
    if (i > 0) {
        return vrai_main(so);
    }
    return 0;
}
