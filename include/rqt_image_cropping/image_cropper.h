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

#ifndef rqt_image_Enhancer__ImageCropper_H
#define rqt_image_Enhancer__ImageCropper_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_image_cropper.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <opencv2/core/core.hpp>

#include <QImage>
#include <QList>
#include <QMutex>
#include <QString>
#include <QSize>
#include <QWidget>

#include <vector>

namespace rqt_image_cropping {

class ImageCropper
  : public rqt_gui_cpp::Plugin
{

  Q_OBJECT

public:

  ImageCropper();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);

  virtual bool eventFilter(QObject* watched, QEvent* event);

  virtual void shutdownPlugin();

  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  void updateDynParam(std::string parameter_name, int value, int min, int max);

  void updatePanAndTiltTakingIntoAccountZoom(int pan, int tilt);

protected slots:

  virtual void updateTopicList();

protected:

  virtual QList<QString> getTopicList(const QSet<QString>& message_types, const QList<QString>& transports);

  virtual void selectTopic(const QString& topic);

protected slots:

  virtual void onInTopicChanged(int index);

  virtual void onZoomEvent(int numDegrees);

  virtual void onLeftClickEvent(QPoint pos);

protected:

  virtual void callbackImage(const sensor_msgs::Image::ConstPtr& msg);

  Ui::ImageCropperWidget ui_;

  QWidget* widget_;

  image_transport::Subscriber subscriber_;

  sensor_msgs::Image::ConstPtr sens_msg_image_;
  sensor_msgs::CameraInfoConstPtr camera_info_;

  double image_min_value_;
  double image_max_value_;

  QImage qimage_;
  QImage selected_region_;
  QMutex qimage_mutex_;

  cv::Mat conversion_mat_;

  QRectF selection_;

  QPointF selection_top_left_;
  QPointF selection_top_left_rect_;
  QSizeF selection_size_;
  QSizeF selection_size_rect_;


};

}

#endif // rqt_image_enhancer__ImageCropper_H
