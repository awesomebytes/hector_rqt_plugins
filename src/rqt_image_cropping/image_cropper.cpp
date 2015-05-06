/*
 * Copyright (c) 2011, Dirk Thomas, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rqt_image_cropping/image_cropper.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <ros/console.h>
#include <sensor_msgs/image_encodings.h>
//imports of shame
#include <sstream>
#include <boost/algorithm/string.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <QMessageBox>
#include <QPainter>

#include <algorithm>

namespace rqt_image_cropping {

ImageCropper::ImageCropper()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  setObjectName("ImageCropper");
}

void ImageCropper::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  ui_.image_frame->installEventFilter(this);

  updateTopicList();

  ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText(""));
  connect(ui_.topics_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(onInTopicChanged(int)));

  ui_.refresh_topics_push_button->setIcon(QIcon::fromTheme("view-refresh"));
  connect(ui_.refresh_topics_push_button, SIGNAL(pressed()), this, SLOT(updateTopicList()));
  

  connect(ui_.image_frame, SIGNAL(leftClickSignal(QPoint)), this, SLOT(onLeftClickEvent(QPoint)));
  connect(ui_.image_frame, SIGNAL(zoomSignal(int)), this, SLOT(onZoomEvent(int))); // so much c++ hate
}

bool ImageCropper::eventFilter(QObject* watched, QEvent* event)
{
  if (watched == ui_.image_frame && event->type() == QEvent::Paint)
  {
    QPainter painter(ui_.image_frame);
    if (!qimage_.isNull())
    {
      ui_.image_frame->resizeToFitAspectRatio();
      // TODO: check if full draw is really necessary
      //QPaintEvent* paint_event = dynamic_cast<QPaintEvent*>(event);
      //painter.drawImage(paint_event->rect(), qimage_);
      qimage_mutex_.lock();
      painter.drawImage(ui_.image_frame->contentsRect(), qimage_);
      qimage_mutex_.unlock();
    } else {
      // default image with gradient
      QLinearGradient gradient(0, 0, ui_.image_frame->frameRect().width(), ui_.image_frame->frameRect().height());
      gradient.setColorAt(0, Qt::white);
      gradient.setColorAt(1, Qt::black);
      painter.setBrush(gradient);
      painter.drawRect(0, 0, ui_.image_frame->frameRect().width() + 1, ui_.image_frame->frameRect().height() + 1);
    }

    ui_.image_frame->update();

    return false;
  }

  return QObject::eventFilter(watched, event);
}

void ImageCropper::shutdownPlugin()
{
  subscriber_.shutdown();
}

void ImageCropper::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  QString topic = ui_.topics_combo_box->currentText();
  //qDebug("ImageCropper::saveSettings() topic '%s'", topic.toStdString().c_str());
  instance_settings.setValue("topic", topic);
}

void ImageCropper::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  QString topic = instance_settings.value("topic", "").toString();
  //qDebug("ImageCropper::restoreSettings() topic '%s'", topic.toStdString().c_str());
  selectTopic(topic);
}

void ImageCropper::updateTopicList()
{
  QSet<QString> message_types;
  message_types.insert("sensor_msgs/Image");

  // get declared transports
  QList<QString> transports;
  image_transport::ImageTransport it(getNodeHandle());
  std::vector<std::string> declared = it.getDeclaredTransports();
  for (std::vector<std::string>::const_iterator it = declared.begin(); it != declared.end(); it++)
  {
    //qDebug("ImageCropper::updateTopicList() declared transport '%s'", it->c_str());
    QString transport = it->c_str();

    // strip prefix from transport name
    QString prefix = "image_transport/";
    if (transport.startsWith(prefix))
    {
      transport = transport.mid(prefix.length());
    }
    transports.append(transport);
  }

  QString selected = ui_.topics_combo_box->currentText();

  // fill combo box
  QList<QString> topics = getTopicList(message_types, transports);
  topics.append("");
  qSort(topics);
  ui_.topics_combo_box->clear();
  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");
    ui_.topics_combo_box->addItem(label, QVariant(*it));
  }

  // restore previous selection
  selectTopic(selected);
}

QList<QString> ImageCropper::getTopicList(const QSet<QString>& message_types, const QList<QString>& transports)
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);

  QSet<QString> all_topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    all_topics.insert(it->name.c_str());
  }

  QList<QString> topics;
  for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin(); it != topic_info.end(); it++)
  {
    if (message_types.contains(it->datatype.c_str()))
    {
      QString topic = it->name.c_str();

      // add raw topic
      topics.append(topic);
      //qDebug("ImageCropper::getTopicList() raw topic '%s'", topic.toStdString().c_str());
      
      // add transport specific sub-topics
      for (QList<QString>::const_iterator jt = transports.begin(); jt != transports.end(); jt++)
      {
        if (all_topics.contains(topic + "/" + *jt))
        {
          QString sub = topic + " " + *jt;
          topics.append(sub);
          //qDebug("ImageCropper::getTopicList() transport specific sub-topic '%s'", sub.toStdString().c_str());
        }
      }
    }
  }
  return topics;
}

void ImageCropper::selectTopic(const QString& topic)
{
  int index = ui_.topics_combo_box->findText(topic);
  if (index == -1)
  {
    index = ui_.topics_combo_box->findText("");
  }
  ui_.topics_combo_box->setCurrentIndex(index);
}

void ImageCropper::onInTopicChanged(int index)
{
  subscriber_.shutdown();

  // reset image on topic change
  qimage_ = QImage();
  ui_.image_frame->update();

  QStringList parts = ui_.topics_combo_box->itemData(index).toString().split(" ");
  QString topic = parts.first();
  QString transport = parts.length() == 2 ? parts.last() : "raw";

  if (!topic.isEmpty())
  {
    image_transport::ImageTransport it(getNodeHandle());
    image_transport::TransportHints hints(transport.toStdString());
    try {
      subscriber_ = it.subscribe(topic.toStdString(), 1, &ImageCropper::callbackImage, this, hints);
      //qDebug("ImageCropper::onInTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
    } catch (image_transport::TransportLoadException& e) {
      QMessageBox::warning(widget_, tr("Loading image transport plugin failed"), e.what());
    }
  }

}

/*
given a parameter name (pan, tilt, zoom)
the value to add/substract to the current value of the parameter
and the minimum and maximum of the value to be finally sent
it sets that dyn param With the recommended way of the ROS docu:
http://wiki.ros.org/hokuyo_node/Tutorials/UsingDynparamToChangeHokuyoLaserParameters
*/
void ImageCropper::updateDynParam(std::string parameter_name, int value, int min, int max){
    if (value == 0){
        std::cout << "asking to set value 0 for " << parameter_name << ", ignoring it" << std::endl;
        return;
    }

    std::string topic_str = subscriber_.getTopic();
    std::vector<std::string> strs;
    boost::split(strs, topic_str, boost::is_any_of("/"));
    //std::cout << "CURRENT TOPIC: " << subscriber_.getTopic() << ", splitted: '" << strs[0] << "' '" << strs[1] << "' "<< std::endl;
    std::string nodepath = "/" + strs[1] + "/axis_camera_ptz"; // that's the dynamic reconfigure of the ptz
    std::string parampath = nodepath + "/" + parameter_name;
    int curr_param_val;
    if (ros::param::get(parampath, curr_param_val)) //construct from the topic name the dynparam/param
    {
        curr_param_val += value;
        if (curr_param_val < min)
            curr_param_val = min;
        if (curr_param_val > max)
            curr_param_val = max;
        std::ostringstream command;
        command << "rosrun dynamic_reconfigure dynparam set " << nodepath << " " << parameter_name << " " << curr_param_val << " &";
        std::cout << "Sending command: " << command.str() << std::endl;
        system(command.str().c_str());
    }
    else
        std::cout << "No param: '" << parampath << "'' found :(" << std::endl;

}

void ImageCropper::updatePanAndTiltTakingIntoAccountZoom(int pan, int tilt){
    std::string topic_str = subscriber_.getTopic();
    std::vector<std::string> strs;
    boost::split(strs, topic_str, boost::is_any_of("/"));
    //std::cout << "CURRENT TOPIC: " << subscriber_.getTopic() << ", splitted: '" << strs[0] << "' '" << strs[1] << "' "<< std::endl;
    std::string nodepath = "/" + strs[1] + "/axis_camera_ptz"; // that's the dynamic reconfigure of the ptz
    std::string parampath = nodepath + "/zoom";
    int curr_zoom;
    if (ros::param::get(parampath, curr_zoom)) //construct from the topic name the dynparam/param
    {
        double multipl = 1.0 - (double)(curr_zoom / 9999.0); // the more zoom, the less degrees we should move
        // set the dynparam
        double adapted_pan = (double) pan * multipl;
        if (adapted_pan < 1.0 && adapted_pan >= 0.0)
            adapted_pan = 1.0;
        else if (adapted_pan > -1.0 && adapted_pan <= 0.0)
            adapted_pan = -1.0;
        updateDynParam("pan", (int)adapted_pan, -180, 180);

        // set the dynparam
        double adapted_tilt = (double) tilt * multipl;
        if (adapted_tilt < 1.0 && adapted_tilt >= 0.0)
            adapted_tilt = 1.0;
        else if (adapted_tilt > -1.0 && adapted_tilt <= 0.0)
            adapted_tilt = -1.0;
        updateDynParam("tilt", (int)adapted_tilt, -90, 0);
    }
}


void ImageCropper::onZoomEvent(int numDegrees)
{
    int finalzoom = 0;
    int zoom_step = 1500;
    finalzoom = (numDegrees / 120) * zoom_step; // qt always gives multiples of 120
    // set the dynparam
    updateDynParam("zoom", finalzoom, 1, 9999);
}

void ImageCropper::onLeftClickEvent(QPoint pos)
{
    //std::cout << "Left click at:\nrx,ry: " << pos.rx() << ", " << pos.ry() << ", \nx,y: " << pos.x() << ", " << pos.y() << std::endl;
    // taken looking at the implementation of the axis camera ptz node
    // get the current pan from the dynamic reconfigure and add/substract a step based on how far the user click from the center
    int panstep = 15;
    int tiltstep = 5;
    //std::cout << "ui_.image_frame->frameRect().width():" << ui_.image_frame->frameRect().width() << std::endl;
    //std::cout << "ui_.image_frame->frameRect().height():" << ui_.image_frame->frameRect().height() << std::endl;
    double relpan = (double) pos.x() / ui_.image_frame->frameRect().width();
    double reltilt = (double) pos.y() / ui_.image_frame->frameRect().height();
    //std::cout << "relpan: " << relpan << ", reltilt: " << reltilt << std::endl;
    int final_pan, final_tilt;
    final_pan = final_tilt = 0;
    //relpan = 1 - relpan;
    if (relpan >= 0.6 && relpan <= 1.0)
        final_pan = panstep;
    else if (relpan <= 0.4 && relpan >= 0.0)
        final_pan = -panstep;
    else
        //std::cout << "relpan outside of the range 0.0 - 1.0, this shouldnt happen!" << std::endl;
        std::cout << "clicking in the center in relation to pan, not moving" << std::endl;

    if (reltilt >= 0.6 && reltilt <= 1.0)
        final_tilt = -tiltstep;
    else if (reltilt <= 0.4 && reltilt >= 0.0)
        final_tilt = tiltstep;
    else
        //std::cout << "reltilt outside of the range 0.0 - 1.0, this shouldnt happen!" << std::endl;
        std::cout << "clicking in the center in relation to tilt, not moving" << std::endl;

    //std::cout << "pan: " << final_pan << " tilt: " << final_tilt << std::endl;

    // set the dynparam
    updatePanAndTiltTakingIntoAccountZoom(final_pan, final_tilt);
//    updateDynParam("pan", final_pan, -180, 180);

//    // set the dynparam
//    updateDynParam("tilt", final_tilt, -90, 0);

}


void ImageCropper::callbackImage(const sensor_msgs::Image::ConstPtr& img)//, const sensor_msgs::CameraInfoConstPtr& ci)
{
        sens_msg_image_ = img;
        //camera_info_ = ci;

        try
        {
            // First let cv_bridge do its magic
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::RGB8);
            conversion_mat_ = cv_ptr->image;
        }
        catch (cv_bridge::Exception& e)
        {
            // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(img);
            if (img->encoding == "CV_8UC3")
            {
                // assuming it is rgb
                conversion_mat_ = cv_ptr->image;
            } else if (img->encoding == "8UC1") {
                // convert gray to rgb
                cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
            } else {
                qWarning("ImageCropper.callback_image() could not convert image from '%s' to 'rgb8' (%s)", img->encoding.c_str(), e.what());
                qimage_ = QImage();
                return;
            }
        }

        // copy temporary image as it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
        QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, QImage::Format_RGB888);
        qimage_mutex_.lock();
        qimage_ = image.copy();
        qimage_mutex_.unlock();

        ui_.image_frame->setAspectRatio(qimage_.width(), qimage_.height());
        //onZoom1(false);

        ui_.image_frame->setInnerFrameMinimumSize(QSize(80, 60));
        ui_.image_frame->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
        widget_->setMinimumSize(QSize(80, 60));
        widget_->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
}

}

PLUGINLIB_EXPORT_CLASS(rqt_image_cropping::ImageCropper, rqt_gui_cpp::Plugin)
