/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
// Authors:
//  * Anatoly Baksheev, Itseez Inc.  myname.mysurname <> mycompany.com
//
//M*/

#ifndef __vtkCloudColorMatSource_h
#define __vtkCloudColorMatSource_h

#include <opencv2/core/mat.hpp>
#include <vtkPolyDataAlgorithm.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>

namespace cv
{
    namespace viz
    {
        class vtkCloudColorMatSource : public vtkPolyDataAlgorithm
        {
        public:
            static vtkCloudColorMatSource *New();
            vtkTypeMacro(vtkCloudColorMatSource,vtkPolyDataAlgorithm);

            virtual void SetCloud(const Mat& cloud);
            virtual void SetColors(const Mat &colors, const Mat &cloud_mask);

        protected:
            vtkCloudColorMatSource();
            ~vtkCloudColorMatSource();

            int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

            vtkSmartPointer<vtkPoints> points;
            vtkSmartPointer<vtkCellArray> vertices;
            vtkSmartPointer<vtkUnsignedCharArray> scalars;
        private:
            vtkCloudColorMatSource(const vtkCloudColorMatSource&);  // Not implemented.
            void operator=(const vtkCloudColorMatSource&);  // Not implemented.

            template<typename _Tp> int filterNanCopy(const Mat& source, int dataType);

            template<typename _Msk, class _NanPred>
            void filterNanCopy(const Mat& colors, const Mat& mask);
        };
    }
}

#endif
