//// Copyright (C) 2007 by Cristóbal Carnero Liñán
//// grendel.ccl@gmail.com
////
//// This file is part of cvBlob.
////
//// cvBlob is free software: you can redistribute it and/or modify
//// it under the terms of the Lesser GNU General Public License as published by
//// the Free Software Foundation, either version 3 of the License, or
//// (at your option) any later version.
////
//// cvBlob is distributed in the hope that it will be useful,
//// but WITHOUT ANY WARRANTY; without even the implied warranty of
//// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//// Lesser GNU General Public License for more details.
////
//// You should have received a copy of the Lesser GNU General Public License
//// along with cvBlob.  If not, see <http://www.gnu.org/licenses/>.
////
//
//#include <cmath>
//#include <iostream>
//#include <sstream>
//using namespace std;
//
//#if (defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__) || (defined(__APPLE__) & defined(__MACH__)))
//#include <cv.h>
//#else
//#include <opencv/cv.h>
//#endif
//
//#include "cvblob.h"
//
//namespace cvb
//{
//
//	std::vector<double> distantBlobTrack(CvBlob const *b, CvTrack const *t){
//		std::vector<double> dd;
//		double d1;
//		if (b->centroid.x < t->minx){
//			if (b->centroid.y < t->miny)
//				//d1 = sqrt((double)(t->minx - b->centroid.x) * (double)(t->minx - b->centroid.x) + (double)(t->miny - b->centroid.y) * (double)(t->miny - b->centroid.y));
//				d1 = MAX(t->minx - b->centroid.x, t->miny - b->centroid.y);
//			else if (b->centroid.y>t->maxy)
//				//d1 = sqrt((double)(t->minx - b->centroid.x) * (double)(t->minx - b->centroid.x) + (double)(b->centroid.y - t->maxy) * (double)(b->centroid.y - t->maxy));
//				d1 = MAX(t->minx - b->centroid.x, b->centroid.y - t->maxy);
//			else // if (t->miny < b->centroid.y)&&(b->centroid.y < t->maxy)
//				d1 = t->minx - b->centroid.x;
//		}
//		else if (b->centroid.x > t->maxx){
//			if (b->centroid.y < t->miny)
//				//d1 = sqrt((double)(b->centroid.x - t->maxx) * (double)(b->centroid.x - t->maxx) + (double)(t->miny - b->centroid.y) * (double)(t->miny - b->centroid.y));
//				d1 = MAX(b->centroid.x - t->maxx, t->miny - b->centroid.y);
//			else if (b->centroid.y > t->maxy)
//				//d1 = sqrt((double)(b->centroid.x - t->maxx) * (double)(b->centroid.x - t->maxx) + (double)(b->centroid.y - t->maxy) * (double)(b->centroid.y - t->maxy));
//				d1 = MAX(b->centroid.x - t->maxx, b->centroid.y - t->maxy);
//			else
//				d1 = b->centroid.x - t->maxx;
//		}
//		else{ // if (t->minx =< b->centroid.x) && (b->centroid.x =< t->maxx){
//			if (b->centroid.y < t->miny)
//				d1 = t->miny - b->centroid.y;
//			else if (b->centroid.y > t->maxy)
//				d1 = b->centroid.y - t->maxy;
//			else
//				d1 = 0.;
//		}
//
//		double d2;
//		if (t->centroid.x < b->minx){
//			if (t->centroid.y < b->miny)
//				//d2 = sqrt((double)(b->minx - t->centroid.x) * (double)(b->minx - t->centroid.x) + (double)(b->miny - t->centroid.y) * (double)(b->miny - t->centroid.y));
//				d2 = MAX(b->minx - t->centroid.x, b->miny - t->centroid.y);
//			else if (t->centroid.y > b->maxy)
//				//d2 = sqrt((double)(b->minx - t->centroid.x) * (double)(b->minx - t->centroid.x) + (double)(t->centroid.y - b->maxy) * (double)(t->centroid.y - b->maxy));
//				d2 = MAX(b->minx - t->centroid.x, t->centroid.y - b->maxy);
//			else // if (b->miny < t->centroid.y)&&(t->centroid.y < b->maxy)
//				d2 = b->minx - t->centroid.x;
//		}
//		else if (t->centroid.x > b->maxx){
//			if (t->centroid.y < b->miny)
//				//d2 = sqrt((double)(t->centroid.x - b->maxx) * (double)(t->centroid.x - b->maxx) + (double)(b->miny - t->centroid.y) * (double)(b->miny - t->centroid.y));
//				d2 = MAX(t->centroid.x - b->maxx, b->miny - t->centroid.y);
//			else if (t->centroid.y > b->maxy)
//				//d2 = sqrt((double)(t->centroid.x - b->maxx) * (double)(t->centroid.x - b->maxx) + (double)(t->centroid.y - b->maxy) * (double)(t->centroid.y - b->maxy));
//				d2 = MAX(t->centroid.x - b->maxx, t->centroid.y - b->maxy);
//			else
//				d2 = t->centroid.x - b->maxx;
//		}
//		else{ // if (b->minx =< t->centroid.x) && (t->centroid.x =< b->maxx){
//			if (t->centroid.y < b->miny)
//				d2 = b->miny - t->centroid.y;
//			else if (t->centroid.y > b->maxy)
//				d2 = t->centroid.y - b->maxy;
//			else
//				d2 = 0.;
//		}
//
//		//return MIN(d1, d2);
//		double d3 = sqrt((double)(t->centroid.x - b->centroid.x) * (double)(t->centroid.x - b->centroid.x) + (double)(t->centroid.y - b->centroid.y) * (double)(t->centroid.y - b->centroid.y));
//		dd.push_back(MIN(d1, d2));
//		dd.push_back(d3);
//		return dd;
//	}
//
//	// Access to matrix
//#define C(blob, track) close[((blob) + (track)*(nBlobs+2))]
//	// Access to accumulators
//#define AB(label) C((label), (nTracks))
//#define AT(id) C((nBlobs), (id))
//	// Access to identifications
//#define IB(label) C((label), (nTracks)+1)
//#define IT(id) C((nBlobs)+1, (id))
//	// Access to registers
//#define B(label) blobs.find(IB(label))->second
//#define T(id) tracks.find(IT(id))->second
//
//	void getClusterForTrack(unsigned int trackPos, CvID *close, unsigned int nBlobs, unsigned int nTracks, CvBlobs const &blobs, CvTracks const &tracks, list<CvBlob*> &bb, list<CvTrack*> &tt);
//	void getClusterForBlob(unsigned int blobPos, CvID *close, unsigned int nBlobs, unsigned int nTracks, CvBlobs const &blobs, CvTracks const &tracks, list<CvBlob*> &bb, list<CvTrack*> &tt){
//		for (unsigned int j = 0; j<nTracks; j++){
//			if (C(blobPos, j)){
//				tt.push_back(T(j));
//
//				unsigned int c = AT(j);
//
//				C(blobPos, j) = 0;
//				AB(blobPos)--;
//				AT(j)--;
//
//				if (c>1){
//					getClusterForTrack(j, close, nBlobs, nTracks, blobs, tracks, bb, tt);
//				}
//			}
//		}
//	}
//
//	void getClusterForTrack(unsigned int trackPos, CvID *close, unsigned int nBlobs, unsigned int nTracks, CvBlobs const &blobs, CvTracks const &tracks, list<CvBlob*> &bb, list<CvTrack*> &tt){
//		for (unsigned int i = 0; i<nBlobs; i++){
//			if (C(i, trackPos)){
//				bb.push_back(B(i));
//
//				unsigned int c = AB(i);
//
//				C(i, trackPos) = 0;
//				AB(i)--;
//				AT(trackPos)--;
//
//				if (c>1){
//					getClusterForBlob(i, close, nBlobs, nTracks, blobs, tracks, bb, tt);
//				}
//			}
//		}
//	}
//
//	void cvUpdateTracks(CvBlobs const &blobs, CvTracks &tracks, double thDistance, unsigned int thInactive, const unsigned int thActive){
//		CV_FUNCNAME("cvUpdateTracks");
//		__CV_BEGIN__;
//
//		unsigned int nBlobs = blobs.size();
//		unsigned int nTracks = tracks.size();
//
//		// Proximity matrix:
//		// Last row/column is for ID/label.
//		// Last-1 "/" is for accumulation.
//		CvID *close = new unsigned int[(nBlobs + 2)*(nTracks + 2)]; // XXX Must be same type than CvLabel.
//
//		try
//		{
//			// Initialization:
//			unsigned int i = 0;
//			unsigned int j = 0;
//			//cout << "=============" << endl;
//			for (int i = 0; i < nBlobs + 2; i++){
//				for (int j = 0; j < nTracks + 2; j++){
//					C(i, j) = 0;
//					//cout << C(i, j) << " , ";
//				}
//				//cout << endl;
//			}
//
//			for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it, i++){
//				AB(i) = 0;
//				IB(i) = it->second->label;
//			}
//
//			CvID maxTrackID = 0;
//
//
//			for (CvTracks::const_iterator jt = tracks.begin(); jt != tracks.end(); ++jt, j++){
//				AT(j) = 0;
//				IT(j) = jt->second->id;
//				if (jt->second->id > maxTrackID)
//					maxTrackID = jt->second->id;
//			}
//
//			// Proximity matrix calculation and "used blob" list initialization:
//			for (i = 0; i < nBlobs; i++){
//				float sdd;
//				int tmpj;
//				if (nTracks == 0)
//					break;
//				else
//				{
//					std::vector<double> dd = distantBlobTrack(B(i), T(0));
//					sdd = dd[1];
//					tmpj = 0;
//				}
//				for (j = 1; j < nTracks; j++){
//					//float v_posn_track = ((T(j)->centroid.y - 193)*1.0*(704 - 137)) - ((T(j)->centroid.x - 137)*1.0*(342 - 193));
//					//float v_posn_blob = ((B(i)->centroid.y - 193)*1.0*(704 - 137)) - ((B(i)->centroid.x - 137)*1.0*(342 - 193));
//					if (distantBlobTrack(B(i), T(j))[1] < sdd){
//						sdd = distantBlobTrack(B(i), T(j))[1];
//						tmpj = j;
//					}
//				}
//				C(i, tmpj) = (sdd < thDistance);
//				if (C(i, tmpj)){
//					AB(i)++;
//					AT(tmpj)++;
//				}
//				//cout << "tmpj: " << tmpj << " , sdd: " << sdd << " , C(i, j)" << C(i, tmpj) << endl;
//			}
//
//			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//			// Detect inactive tracks
//			for (j = 0; j < nTracks; j++){
//				unsigned int c = AT(j);
//
//				if (c == 0){
//					//cout << "Inactive track: " << j << endl;
//
//					// Inactive track.
//					CvTrack *track = T(j);
//					track->inactive++;
//					track->label = 0;
//				}
//			}
//			//cout << "--------------" << endl;
//			//for (i = 0; i < nBlobs + 2; i++){
//			//	for (j = 0; j < nTracks + 2; j++){
//			//		cout << C(i, j) << " , ";
//			//	}
//			//	cout << endl;
//			//}
//
//			// Detect new tracks
//			for (i = 0; i < nBlobs; i++){
//				unsigned int c = AB(i);
//
//				if (c == 0){
//					//cout << "Blob (new track): " << maxTrackID+1 << endl;
//					//cout << *B(i) << endl;
//
//					// New track.
//					maxTrackID = (maxTrackID + 1) % 1000;
//					CvBlob *blob = B(i);
//					CvTrack *track = new CvTrack;
//					track->id = maxTrackID;
//					track->label = blob->label;
//					track->minx = blob->minx;
//					track->miny = blob->miny;
//					track->maxx = blob->maxx;
//					track->maxy = blob->maxy;
//					track->centroid = blob->centroid;
//					track->pre_centroid = track->centroid;
//					track->lifetime = 0;
//					track->active = 0;
//					track->inactive = 0;
//					track->pe_time = 0;
//					tracks.insert(CvIDTrack(maxTrackID, track));
//				}
//			}
//
//			// Update track
//			for (j = 0; j < nTracks; j++){
//				if (AT(j)){
//					CvTrack *track = T(j);
//					CvBlob *blob = nullptr;
//					for (i = 0; i < nBlobs; i++){
//						float area = 0;
//						if (C(i, j)){
//							if (B(i)->area>area){
//								area = B(i)->area;
//								blob = B(i);
//							}
//						}
//					}
//
//					//cout << "Matching: track=" << track->id << ", blob=" << blob->label << endl;
//					track->label = blob->label;
//					track->pre_centroid = track->centroid;
//					track->centroid = blob->centroid;
//					track->minx = blob->minx;
//					track->miny = blob->miny;
//					track->maxx = blob->maxx;
//					track->maxy = blob->maxy;
//					if (track->inactive){
//						track->active = 0;
//					}
//					track->inactive = 0;
//				}
//			}
//
//			// Clustering
//			//for (j = 0; j < nTracks; j++){
//			//	unsigned int c = AT(j);
//
//			//	if (c){
//			//		list<CvTrack*> tt; tt.push_back(T(j));
//			//		list<CvBlob*> bb;
//
//			//		getClusterForTrack(j, close, nBlobs, nTracks, blobs, tracks, bb, tt);
//
//			//		// Select track
//			//		CvTrack *track = nullptr;
//			//		unsigned int area = 0;
//			//		for (list<CvTrack*>::const_iterator it = tt.begin(); it != tt.end(); ++it){
//			//			CvTrack *t = *it;
//
//			//			unsigned int a = (t->maxx - t->minx)*(t->maxy - t->miny);
//			//			if (a > area){
//			//				area = a;
//			//				track = t;
//			//			}
//			//		}
//
//			//		// Select blob
//			//		CvBlob *blob = nullptr;
//			//		area = 0;
//			//		//cout << "Matching blobs: ";
//			//		for (list<CvBlob*>::const_iterator it = bb.begin(); it != bb.end(); ++it){
//			//			CvBlob *b = *it;
//
//			//			//cout << b->label << " ";
//
//			//			if (b->area > area){
//			//				area = b->area;
//			//				blob = b;
//			//			}
//			//		}
//			//		//cout << endl;
//
//			//		// Update track
//			//		//cout << "Matching: track=" << track->id << ", blob=" << blob->label << endl;
//			//		track->label = blob->label;
//			//		track->centroid = blob->centroid;
//			//		track->minx = blob->minx;
//			//		track->miny = blob->miny;
//			//		track->maxx = blob->maxx;
//			//		track->maxy = blob->maxy;
//			//		if (track->inactive){
//			//			track->active = 0;
//			//		}
//			//		track->inactive = 0;
//
//			//		// Others to inactive
//			//		for (list<CvTrack*>::const_iterator it = tt.begin(); it != tt.end(); ++it){
//			//			CvTrack *t = *it;
//
//			//			if (t != track){
//			//				//cout << "Inactive: track=" << t->id << endl;
//			//				t->inactive++;
//			//				t->label = 0;
//			//			}
//			//		}
//			//	}
//			//}
//			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//			for (CvTracks::iterator jt = tracks.begin(); jt != tracks.end();){
//				if ((jt->second->inactive >= thInactive) || ((jt->second->inactive) && (thActive) && (jt->second->active < thActive))){
//					delete jt->second;
//					tracks.erase(jt++);
//				}
//				else{
//					jt->second->lifetime++;
//					if (!jt->second->inactive){
//						jt->second->active++;
//					}
//					++jt;
//				}
//			}
//		}
//		catch (...){
//			delete[] close;
//			throw; // TODO: OpenCV style.
//		}
//
//		delete[] close;
//
//		__CV_END__;
//	}
//
//	CvFont *defaultFont = NULL;
//
//	void cvRenderTracks(CvTracks const tracks, IplImage *imgSource, IplImage *imgDest, unsigned short mode, CvFont *font){
//		CV_FUNCNAME("cvRenderTracks");
//		__CV_BEGIN__;
//
//		// CV_ASSERT(imgDest&&(imgDest->depth==IPL_DEPTH_8U)&&(imgDest->nChannels==3)); -------------------------------------------------------------------------------------------------------------
//
//		if ((mode&CV_TRACK_RENDER_ID) && (!font)){
//			if (!defaultFont){
//				font = defaultFont = new CvFont;
//				cvInitFont(font, CV_FONT_HERSHEY_DUPLEX, 0.5, 0.5, 0, 1);
//				// Other fonts:
//				//   CV_FONT_HERSHEY_SIMPLEX, CV_FONT_HERSHEY_PLAIN,
//				//   CV_FONT_HERSHEY_DUPLEX, CV_FONT_HERSHEY_COMPLEX,
//				//   CV_FONT_HERSHEY_TRIPLEX, CV_FONT_HERSHEY_COMPLEX_SMALL,
//				//   CV_FONT_HERSHEY_SCRIPT_SIMPLEX, CV_FONT_HERSHEY_SCRIPT_COMPLEX
//			}
//			else{
//				font = defaultFont;
//			}
//		}
//
//		if (mode){
//			for (CvTracks::const_iterator it = tracks.begin(); it != tracks.end(); ++it){
//				if (mode&CV_TRACK_RENDER_ID){
//					if (!it->second->inactive){
//						stringstream buffer;
//						buffer << it->first;
//						//cvPutText(imgDest, buffer.str().c_str(), cvPoint((int)it->second->centroid.x, (int)it->second->centroid.y), font, CV_RGB(0.,255.,0.));
//					}
//				}
//
//				if (mode&CV_TRACK_RENDER_BOUNDING_BOX){
//					if (it->second->inactive)
//						cvRectangle(imgDest, cvPoint(it->second->minx, it->second->miny), cvPoint(it->second->maxx - 1, it->second->maxy - 1), CV_RGB(0., 0., 255.));
//					else
//						cvRectangle(imgDest, cvPoint(it->second->minx, it->second->miny), cvPoint(it->second->maxx - 1, it->second->maxy - 1), CV_RGB(0., 0., 255.));
//				}
//
//				if (mode&CV_TRACK_RENDER_TO_LOG){
//					clog << "Track " << it->second->id << endl;
//					if (it->second->inactive)
//						clog << " - Inactive for " << it->second->inactive << " frames" << endl;
//					else
//						clog << " - Associated with blob " << it->second->label << endl;
//					clog << " - Lifetime " << it->second->lifetime << endl;
//					clog << " - Active " << it->second->active << endl;
//					clog << " - Bounding box: (" << it->second->minx << ", " << it->second->miny << ") - (" << it->second->maxx << ", " << it->second->maxy << ")" << endl;
//					clog << " - Centroid: (" << it->second->centroid.x << ", " << it->second->centroid.y << ")" << endl;
//					clog << endl;
//				}
//
//				if (mode&CV_TRACK_RENDER_TO_STD){
//					cout << "Track " << it->second->id << endl;
//					if (it->second->inactive)
//						cout << " - Inactive for " << it->second->inactive << " frames" << endl;
//					else
//						cout << " - Associated with blobs " << it->second->label << endl;
//					cout << " - Lifetime " << it->second->lifetime << endl;
//					cout << " - Active " << it->second->active << endl;
//					cout << " - Bounding box: (" << it->second->minx << ", " << it->second->miny << ") - (" << it->second->maxx << ", " << it->second->maxy << ")" << endl;
//					cout << " - Centroid: (" << it->second->centroid.x << ", " << it->second->centroid.y << ")" << endl;
//					cout << endl;
//				}
//			}
//		}
//
//		__CV_END__;
//	}
//	const CvTracks Copy(CvTracks tracks){
//		CvTracks new_tracks;
//		for (std::map<cvb::CvID, cvb::CvTrack*>::iterator it = tracks.begin(); it != tracks.end(); it++)
//		{
//			cvb::CvID id = (*it).first;
//			cvb::CvTrack* track = new CvTrack;
//			track->id = id;
//			track->label = (*it).second->label;
//			track->minx = (*it).second->minx;
//			track->miny = (*it).second->miny;
//			track->maxx = (*it).second->maxx;
//			track->maxy = (*it).second->maxy;
//			track->centroid = (*it).second->centroid;
//			track->lifetime = (*it).second->lifetime;
//			track->active = (*it).second->active;
//			track->inactive = (*it).second->inactive;
//			track->pe_time = (*it).second->pe_time;
//			track->pre_centroid = (*it).second->pre_centroid;
//			new_tracks.insert(CvIDTrack(id, track));
//		}
//		return new_tracks;
//	}
//}
//
//
//
//





/////////////////////////////////////////////////////////////////////////////////////////



// Copyright (C) 2007 by Cristóbal Carnero Liñán
// grendel.ccl@gmail.com
//
// This file is part of cvBlob.
//
// cvBlob is free software: you can redistribute it and/or modify
// it under the terms of the Lesser GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// cvBlob is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// Lesser GNU General Public License for more details.
//
// You should have received a copy of the Lesser GNU General Public License
// along with cvBlob.  If not, see <http://www.gnu.org/licenses/>.
//

#include <cmath>
#include <iostream>
#include <sstream>
using namespace std;

#if (defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__) || (defined(__APPLE__) & defined(__MACH__)))
#include <cv.h>
#else
#include <opencv/cv.h>
#endif

#include "cvblob.h"

namespace cvb
{

	std::vector<double> distantBlobTrack(CvBlob const *b, CvTrack const *t){
		std::vector<double> dd;
		double d1;
		if (b->centroid.x < t->minx){
			if (b->centroid.y < t->miny)
				//d1 = sqrt((double)(t->minx - b->centroid.x) * (double)(t->minx - b->centroid.x) + (double)(t->miny - b->centroid.y) * (double)(t->miny - b->centroid.y));
				d1 = MAX(t->minx - b->centroid.x, t->miny - b->centroid.y);
			else if (b->centroid.y>t->maxy)
				//d1 = sqrt((double)(t->minx - b->centroid.x) * (double)(t->minx - b->centroid.x) + (double)(b->centroid.y - t->maxy) * (double)(b->centroid.y - t->maxy));
				d1 = MAX(t->minx - b->centroid.x, b->centroid.y - t->maxy);
			else // if (t->miny < b->centroid.y)&&(b->centroid.y < t->maxy)
				d1 = t->minx - b->centroid.x;
		}
		else if (b->centroid.x > t->maxx){
			if (b->centroid.y < t->miny)
				//d1 = sqrt((double)(b->centroid.x - t->maxx) * (double)(b->centroid.x - t->maxx) + (double)(t->miny - b->centroid.y) * (double)(t->miny - b->centroid.y));
				d1 = MAX(b->centroid.x - t->maxx, t->miny - b->centroid.y);
			else if (b->centroid.y > t->maxy)
				//d1 = sqrt((double)(b->centroid.x - t->maxx) * (double)(b->centroid.x - t->maxx) + (double)(b->centroid.y - t->maxy) * (double)(b->centroid.y - t->maxy));
				d1 = MAX(b->centroid.x - t->maxx, b->centroid.y - t->maxy);
			else
				d1 = b->centroid.x - t->maxx;
		}
		else{ // if (t->minx =< b->centroid.x) && (b->centroid.x =< t->maxx){
			if (b->centroid.y < t->miny)
				d1 = t->miny - b->centroid.y;
			else if (b->centroid.y > t->maxy)
				d1 = b->centroid.y - t->maxy;
			else
				d1 = 0.;
		}

		double d2;
		if (t->centroid.x < b->minx){
			if (t->centroid.y < b->miny)
				//d2 = sqrt((double)(b->minx - t->centroid.x) * (double)(b->minx - t->centroid.x) + (double)(b->miny - t->centroid.y) * (double)(b->miny - t->centroid.y));
				d2 = MAX(b->minx - t->centroid.x, b->miny - t->centroid.y);
			else if (t->centroid.y > b->maxy)
				//d2 = sqrt((double)(b->minx - t->centroid.x) * (double)(b->minx - t->centroid.x) + (double)(t->centroid.y - b->maxy) * (double)(t->centroid.y - b->maxy));
				d2 = MAX(b->minx - t->centroid.x, t->centroid.y - b->maxy);
			else // if (b->miny < t->centroid.y)&&(t->centroid.y < b->maxy)
				d2 = b->minx - t->centroid.x;
		}
		else if (t->centroid.x > b->maxx){
			if (t->centroid.y < b->miny)
				//d2 = sqrt((double)(t->centroid.x - b->maxx) * (double)(t->centroid.x - b->maxx) + (double)(b->miny - t->centroid.y) * (double)(b->miny - t->centroid.y));
				d2 = MAX(t->centroid.x - b->maxx, b->miny - t->centroid.y);
			else if (t->centroid.y > b->maxy)
				//d2 = sqrt((double)(t->centroid.x - b->maxx) * (double)(t->centroid.x - b->maxx) + (double)(t->centroid.y - b->maxy) * (double)(t->centroid.y - b->maxy));
				d2 = MAX(t->centroid.x - b->maxx, t->centroid.y - b->maxy);
			else
				d2 = t->centroid.x - b->maxx;
		}
		else{ // if (b->minx =< t->centroid.x) && (t->centroid.x =< b->maxx){
			if (t->centroid.y < b->miny)
				d2 = b->miny - t->centroid.y;
			else if (t->centroid.y > b->maxy)
				d2 = t->centroid.y - b->maxy;
			else
				d2 = 0.;
		}

		//return MIN(d1, d2);
		double d3 = sqrt((double)(t->centroid.x - b->centroid.x) * (double)(t->centroid.x - b->centroid.x) + (double)(t->centroid.y - b->centroid.y) * (double)(t->centroid.y - b->centroid.y));
		dd.push_back(MIN(d1, d2));
		dd.push_back(d3);
		return dd;
	}

	// Access to matrix
#define C(blob, track) close[((blob) + (track)*(nBlobs+2))]
	// Access to accumulators
#define AB(label) C((label), (nTracks))
#define AT(id) C((nBlobs), (id))
	// Access to identifications
#define IB(label) C((label), (nTracks)+1)
#define IT(id) C((nBlobs)+1, (id))
	// Access to registers
#define B(label) blobs.find(IB(label))->second
#define T(id) tracks.find(IT(id))->second

	void getClusterForTrack(unsigned int trackPos, CvID *close, unsigned int nBlobs, unsigned int nTracks, CvBlobs const &blobs, CvTracks const &tracks, list<CvBlob*> &bb, list<CvTrack*> &tt);
	void calculateMovement(cvb::CvTrack* track, short const length, float* degree, float* mag);
	void getClusterForBlob(unsigned int blobPos, CvID *close, unsigned int nBlobs, unsigned int nTracks, CvBlobs const &blobs, CvTracks const &tracks, list<CvBlob*> &bb, list<CvTrack*> &tt){
		for (unsigned int j = 0; j<nTracks; j++){
			if (C(blobPos, j)){
				tt.push_back(T(j));

				unsigned int c = AT(j);

				C(blobPos, j) = 0;
				AB(blobPos)--;
				AT(j)--;

				if (c>1){
					getClusterForTrack(j, close, nBlobs, nTracks, blobs, tracks, bb, tt);
				}
			}
		}
	}

	void getClusterForTrack(unsigned int trackPos, CvID *close, unsigned int nBlobs, unsigned int nTracks, CvBlobs const &blobs, CvTracks const &tracks, list<CvBlob*> &bb, list<CvTrack*> &tt){
		for (unsigned int i = 0; i<nBlobs; i++){
			if (C(i, trackPos)){
				bb.push_back(B(i));

				unsigned int c = AB(i);

				C(i, trackPos) = 0;
				AB(i)--;
				AT(trackPos)--;

				if (c>1){
					getClusterForBlob(i, close, nBlobs, nTracks, blobs, tracks, bb, tt);
				}
			}
		}
	}

	void cvUpdateTracks(cv::Mat frame, CvBlobs const &blobs, CvTracks &tracks, const double thDistance, const unsigned int thInactive, const unsigned int thActive){
		CV_FUNCNAME("cvUpdateTracks");
		__CV_BEGIN__;

		unsigned int nBlobs = blobs.size();
		unsigned int nTracks = tracks.size();

		// Proximity matrix:
		// Last row/column is for ID/label.
		// Last-1 "/" is for accumulation.
		CvID *close = new unsigned int[(nBlobs + 2)*(nTracks + 2)]; // XXX Must be same type than CvLabel.

		try
		{
			// Initialization:
			unsigned int i = 0;
			for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it, i++){
				AB(i) = 0;
				IB(i) = it->second->label;
			}

			CvID maxTrackID = 0;

			unsigned int j = 0;
			for (CvTracks::const_iterator jt = tracks.begin(); jt != tracks.end(); ++jt, j++){
				AT(j) = 0;
				IT(j) = jt->second->id;
				if (jt->second->id > maxTrackID)
					maxTrackID = jt->second->id;
			}

			// Proximity matrix calculation and "used blob" list initialization:
			for (i = 0; i < nBlobs; i++){
				for (j = 0; j < nTracks; j++){
					//float v_posn_track = ((T(j)->centroid.y - 193)*1.0*(704 - 137)) - ((T(j)->centroid.x - 137)*1.0*(342 - 193));
					//float v_posn_blob = ((B(i)->centroid.y - 193)*1.0*(704 - 137)) - ((B(i)->centroid.x - 137)*1.0*(342 - 193));
					std::vector<double> dd = distantBlobTrack(B(i), T(j));



					if (C(i, j) = (dd[0] < thDistance)){
						int tmpj = j;
						for (int n = 0; n < nTracks; n++){
							if (distantBlobTrack(B(i), T(n))[0] < thDistance && distantBlobTrack(B(i), T(n))[1] < dd[1]){
								tmpj = n;
							}
						}
						if (tmpj == j){
							AB(i)++;
							AT(j)++;
						}
						else{
							C(i, j) = !C(i, j);
						}
					}
				}
			}

			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Detect inactive tracks
			for (j = 0; j < nTracks; j++){
				unsigned int c = AT(j);

				if (c == 0){
					//cout << "Inactive track: " << j << endl;

					// Inactive track.
					CvTrack *track = T(j);
					track->inactive++;
					track->label = 0;
				}
			}

			// Detect new tracks
			for (i = 0; i < nBlobs; i++){
				unsigned int c = AB(i);

				if (c == 0){
					//cout << "Blob (new track): " << maxTrackID+1 << endl;
					//cout << *B(i) << endl;

					// New track.
					maxTrackID++;
					CvBlob *blob = B(i);
					CvTrack *track = new CvTrack;
					track->id = maxTrackID;
					track->label = blob->label;
					track->minx = blob->minx;
					track->miny = blob->miny;
					track->maxx = blob->maxx;
					track->maxy = blob->maxy;
					track->centroid = blob->centroid;
					track->pre_centroid = track->centroid;
					track->lifetime = 0;
					track->active = 0;
					track->inactive = 0;
					track->pe_time = 0;
					track->angle = 0.0;
					track->angleMat = cv::Mat::zeros(frame.rows, frame.cols, CV_32FC1);
					track->flowMat = cv::Mat::zeros(frame.rows, frame.cols, CV_32FC2);
					track->motion_mask = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC1);

					track->width = track->maxx - track->minx;
					track->height = track->maxy - track->miny;
					track->area = blob->area;

					//-- angle
					float ang = 0.0, mag = 0.0;
					calculateMovement(track, 14, &ang, &mag);
					track->angles_vector.push_back(ang);
					if (track->angles_vector.size() > 30)
						track->angles_vector.erase(track->angles_vector.begin());
					if (track->angles_vector.size() > 7)
						track->angle = track->angles_vector[track->angles_vector.size() - 7];


					track->trajectory.push_back(cv::Point(track->centroid.x, track->centroid.y));
					if (track->trajectory.size() > 30)
						track->trajectory.erase(track->trajectory.begin());

					tracks.insert(CvIDTrack(maxTrackID, track));
				}
			}

			// Clustering
			for (j = 0; j < nTracks; j++){
				unsigned int c = AT(j);

				if (c){
 					list<CvTrack*> tt; tt.push_back(T(j));
					list<CvBlob*> bb;

					getClusterForTrack(j, close, nBlobs, nTracks, blobs, tracks, bb, tt);

					// Select track
					CvTrack *track = nullptr;
					unsigned int area = 0;
					for (list<CvTrack*>::const_iterator it = tt.begin(); it != tt.end(); ++it){
						CvTrack *t = *it;

						unsigned int a = (t->maxx - t->minx + 1)*(t->maxy - t->miny + 1);
						if (a > area){
							area = a;
							track = t;
						}
					}

					// Select blob
					CvBlob *blob = nullptr;
					area = 0;
					//cout << "Matching blobs: ";
					for (list<CvBlob*>::const_iterator it = bb.begin(); it != bb.end(); ++it){
						CvBlob *b = *it;

						//cout << b->label << " ";

						if (b->area > area){
							area = b->area;
							blob = b;
						}
					}
					//cout << endl;

					// Update track
					//cout << "Matching: track=" << track->id << ", blob=" << blob->label << endl;
					track->label = blob->label;
					track->pre_centroid = track->centroid;
					track->centroid = blob->centroid;
					track->minx = blob->minx;
					track->miny = blob->miny;
					track->maxx = blob->maxx;
					track->maxy = blob->maxy;

					track->width = track->maxx - track->minx;
					track->height = track->maxy - track->miny;
					track->area = blob->area;

					track->trajectory.push_back(cv::Point(track->centroid.x, track->centroid.y));
					if (track->trajectory.size() > 30)
						track->trajectory.erase(track->trajectory.begin());

					//-- angle and magnitude
					float ang = 0.0, mag = 0.0;
					calculateMovement(track, 10, &ang, &mag);
					track->angle = ang;
					track->magnitude = mag;

					track->angles_vector.push_back(ang);
					if (track->angles_vector.size() > 30)
						track->angles_vector.erase(track->angles_vector.begin());
					if (track->angles_vector.size() > 5)
						track->angle = track->angles_vector[track->angles_vector.size() - 5];

					if (track->inactive){
						track->active = 0;
					}
					track->inactive = 0;

					// Others to inactive
					for (list<CvTrack*>::const_iterator it = tt.begin(); it != tt.end(); ++it){
						CvTrack *t = *it;

						if (t != track){
							//cout << "Inactive: track=" << t->id << endl;
							t->inactive++;
							t->label = 0;
						}
					}
				}
			}
			/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			for (CvTracks::iterator jt = tracks.begin(); jt != tracks.end();){
				if ((jt->second->inactive >= thInactive) || ((jt->second->inactive) && (thActive) && (jt->second->active < thActive))){
					delete jt->second;
					tracks.erase(jt++);
				}
				else{
					jt->second->lifetime++;
					if (!jt->second->inactive){
						jt->second->active++;
					}
					++jt;
				}
			}
		}
		catch (...){
			delete[] close;
			throw; // TODO: OpenCV style.
		}

		delete[] close;

		__CV_END__;
	}

/////////////////Modified version


	//void cvUpdateTracks(CvBlobs const &blobs, CvTracks &tracks, double thDistance, unsigned int thInactive, const unsigned int thActive){
	//	CV_FUNCNAME("cvUpdateTracks");
	//	__CV_BEGIN__;

	//	unsigned int nBlobs = blobs.size();
	//	unsigned int nTracks = tracks.size();

	//	// Proximity matrix:
	//	// Last row/column is for ID/label.
	//	// Last-1 "/" is for accumulation.
	//	CvID *close = new unsigned int[(nBlobs + 2)*(nTracks + 2)]; // XXX Must be same type than CvLabel.

	//	try
	//	{
	//		// Initialization:
	//		unsigned int i = 0;
	//		unsigned int j = 0;
	//		//cout << "=============" << endl;
	//		for (int i = 0; i < nBlobs + 2; i++){
	//			for (int j = 0; j < nTracks + 2; j++){
	//				C(i, j) = 0;
	//				//cout << C(i, j) << " , ";
	//			}
	//			//cout << endl;
	//		}

	//		for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it, i++){
	//			AB(i) = 0;
	//			IB(i) = it->second->label;
	//		}

	//		CvID maxTrackID = 0;


	//		for (CvTracks::const_iterator jt = tracks.begin(); jt != tracks.end(); ++jt, j++){
	//			AT(j) = 0;
	//			IT(j) = jt->second->id;
	//			if (jt->second->id > maxTrackID)
	//				maxTrackID = jt->second->id;
	//		}

	//		// Proximity matrix calculation and "used blob" list initialization:
	//		for (i = 0; i < nBlobs; i++){
	//			float sdd;
	//			int tmpj;
	//			if (nTracks == 0)
	//				break;
	//			else
	//			{
	//				std::vector<double> dd = distantBlobTrack(B(i), T(0));
	//				sdd = dd[1];
	//				tmpj = 0;
	//			}
	//			for (j = 1; j < nTracks; j++){
	//				//float v_posn_track = ((T(j)->centroid.y - 193)*1.0*(704 - 137)) - ((T(j)->centroid.x - 137)*1.0*(342 - 193));
	//				//float v_posn_blob = ((B(i)->centroid.y - 193)*1.0*(704 - 137)) - ((B(i)->centroid.x - 137)*1.0*(342 - 193));
	//				if (distantBlobTrack(B(i), T(j))[1] < sdd){
	//					sdd = distantBlobTrack(B(i), T(j))[1];
	//					tmpj = j;
	//				}
	//			}
	//			C(i, tmpj) = (sdd < thDistance);
	//			if (C(i, tmpj)){
	//				AB(i)++;
	//				AT(tmpj)++;
	//			}
	//			//cout << "tmpj: " << tmpj << " , sdd: " << sdd << " , C(i, j)" << C(i, tmpj) << endl;
	//		}

	//		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//		// Detect inactive tracks
	//		for (j = 0; j < nTracks; j++){
	//			unsigned int c = AT(j);

	//			if (c == 0){
	//				//cout << "Inactive track: " << j << endl;

	//				// Inactive track.
	//				CvTrack *track = T(j);
	//				track->inactive++;
	//				track->label = 0;
	//			}
	//		}
	//		//cout << "--------------" << endl;
	//		//for (i = 0; i < nBlobs + 2; i++){
	//		//	for (j = 0; j < nTracks + 2; j++){
	//		//		cout << C(i, j) << " , ";
	//		//	}
	//		//	cout << endl;
	//		//}

	//		// Detect new tracks
	//		for (i = 0; i < nBlobs; i++){
	//			unsigned int c = AB(i);

	//			if (c == 0){
	//				//cout << "Blob (new track): " << maxTrackID+1 << endl;
	//				//cout << *B(i) << endl;

	//				// New track.
	//				maxTrackID = (maxTrackID+1)%1000;
	//				CvBlob *blob = B(i);
	//				CvTrack *track = new CvTrack;
	//				track->id = maxTrackID;
	//				track->label = blob->label;
	//				track->minx = blob->minx;
	//				track->miny = blob->miny;
	//				track->maxx = blob->maxx;
	//				track->maxy = blob->maxy;
	//				track->centroid = blob->centroid;
	//				track->lifetime = 0;
	//				track->active = 0;
	//				track->inactive = 0;
	//				track->pe_time = 0;
	//				tracks.insert(CvIDTrack(maxTrackID, track));
	//			}
	//		}

	//		// Update track
	//		for (j = 0; j < nTracks; j++){
	//			if (AT(j)){
	//				CvTrack *track = T(j);
	//				CvBlob *blob = nullptr;
	//				for (i = 0; i < nBlobs; i++){
	//					float area = 0;
	//					if (C(i, j)){
	//						if (B(i)->area>area){
	//							area = B(i)->area;
	//							blob = B(i);
	//						}
	//					}
	//				}

	//				//cout << "Matching: track=" << track->id << ", blob=" << blob->label << endl;
	//				track->label = blob->label;
	//				track->centroid = blob->centroid;
	//				track->minx = blob->minx;
	//				track->miny = blob->miny;
	//				track->maxx = blob->maxx;
	//				track->maxy = blob->maxy;
	//				if (track->inactive){
	//					track->active = 0;
	//				}
	//				track->inactive = 0;
	//			}
	//		}

	//		// Clustering
	//		//for (j = 0; j < nTracks; j++){
	//		//	unsigned int c = AT(j);

	//		//	if (c){
	//		//		list<CvTrack*> tt; tt.push_back(T(j));
	//		//		list<CvBlob*> bb;

	//		//		getClusterForTrack(j, close, nBlobs, nTracks, blobs, tracks, bb, tt);

	//		//		// Select track
	//		//		CvTrack *track = nullptr;
	//		//		unsigned int area = 0;
	//		//		for (list<CvTrack*>::const_iterator it = tt.begin(); it != tt.end(); ++it){
	//		//			CvTrack *t = *it;

	//		//			unsigned int a = (t->maxx - t->minx)*(t->maxy - t->miny);
	//		//			if (a > area){
	//		//				area = a;
	//		//				track = t;
	//		//			}
	//		//		}

	//		//		// Select blob
	//		//		CvBlob *blob = nullptr;
	//		//		area = 0;
	//		//		//cout << "Matching blobs: ";
	//		//		for (list<CvBlob*>::const_iterator it = bb.begin(); it != bb.end(); ++it){
	//		//			CvBlob *b = *it;

	//		//			//cout << b->label << " ";

	//		//			if (b->area > area){
	//		//				area = b->area;
	//		//				blob = b;
	//		//			}
	//		//		}
	//		//		//cout << endl;

	//		//		// Update track
	//		//		//cout << "Matching: track=" << track->id << ", blob=" << blob->label << endl;
	//		//		track->label = blob->label;
	//		//		track->centroid = blob->centroid;
	//		//		track->minx = blob->minx;
	//		//		track->miny = blob->miny;
	//		//		track->maxx = blob->maxx;
	//		//		track->maxy = blob->maxy;
	//		//		if (track->inactive){
	//		//			track->active = 0;
	//		//		}
	//		//		track->inactive = 0;

	//		//		// Others to inactive
	//		//		for (list<CvTrack*>::const_iterator it = tt.begin(); it != tt.end(); ++it){
	//		//			CvTrack *t = *it;

	//		//			if (t != track){
	//		//				//cout << "Inactive: track=" << t->id << endl;
	//		//				t->inactive++;
	//		//				t->label = 0;
	//		//			}
	//		//		}
	//		//	}
	//		//}
	//		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//		for (CvTracks::iterator jt = tracks.begin(); jt != tracks.end();){
	//			if ((jt->second->inactive >= thInactive) || ((jt->second->inactive) && (thActive) && (jt->second->active < thActive))){
	//				delete jt->second;
	//				tracks.erase(jt++);
	//			}
	//			else{
	//				jt->second->lifetime++;
	//				if (!jt->second->inactive){
	//					jt->second->active++;
	//				}
	//				++jt;
	//			}
	//		}
	//	}
	//	catch (...){
	//		delete[] close;
	//		throw; // TODO: OpenCV style.
	//	}

	//	delete[] close;

	//	__CV_END__;
	//}

	CvFont *defaultFont = NULL;

	void cvRenderTracks(CvTracks const tracks, IplImage *imgSource, IplImage *imgDest, unsigned short mode, CvFont *font){
		CV_FUNCNAME("cvRenderTracks");
		__CV_BEGIN__;

		// CV_ASSERT(imgDest&&(imgDest->depth==IPL_DEPTH_8U)&&(imgDest->nChannels==3)); -------------------------------------------------------------------------------------------------------------

		if ((mode&CV_TRACK_RENDER_ID) && (!font)){
			if (!defaultFont){
				font = defaultFont = new CvFont;
				cvInitFont(font, CV_FONT_HERSHEY_DUPLEX, 0.5, 0.5, 0, 1);
				// Other fonts:
				//   CV_FONT_HERSHEY_SIMPLEX, CV_FONT_HERSHEY_PLAIN,
				//   CV_FONT_HERSHEY_DUPLEX, CV_FONT_HERSHEY_COMPLEX,
				//   CV_FONT_HERSHEY_TRIPLEX, CV_FONT_HERSHEY_COMPLEX_SMALL,
				//   CV_FONT_HERSHEY_SCRIPT_SIMPLEX, CV_FONT_HERSHEY_SCRIPT_COMPLEX
			}
			else{
				font = defaultFont;
			}
		}

		if (mode){
			for (CvTracks::const_iterator it = tracks.begin(); it != tracks.end(); ++it){
				if (mode&CV_TRACK_RENDER_ID){
					if (!it->second->inactive){
						stringstream buffer;
						buffer << it->first;
						cvPutText(imgDest, buffer.str().c_str(), cvPoint((int)it->second->centroid.x, (int)it->second->centroid.y), font, CV_RGB(0.,255.,0.));
					}
				}

				if (mode&CV_TRACK_RENDER_BOUNDING_BOX){
					if (it->second->inactive)
						cvRectangle(imgDest, cvPoint(it->second->minx, it->second->miny), cvPoint(it->second->maxx - 1, it->second->maxy - 1), CV_RGB(0., 0., 255.));
					else
						cvRectangle(imgDest, cvPoint(it->second->minx, it->second->miny), cvPoint(it->second->maxx - 1, it->second->maxy - 1), CV_RGB(0., 0., 255.));
				}

				if (mode&CV_TRACK_RENDER_TO_LOG){
					clog << "Track " << it->second->id << endl;
					if (it->second->inactive)
						clog << " - Inactive for " << it->second->inactive << " frames" << endl;
					else
						clog << " - Associated with blob " << it->second->label << endl;
					clog << " - Lifetime " << it->second->lifetime << endl;
					clog << " - Active " << it->second->active << endl;
					clog << " - Bounding box: (" << it->second->minx << ", " << it->second->miny << ") - (" << it->second->maxx << ", " << it->second->maxy << ")" << endl;
					clog << " - Centroid: (" << it->second->centroid.x << ", " << it->second->centroid.y << ")" << endl;
					clog << endl;
				}

				if (mode&CV_TRACK_RENDER_TO_STD){
					cout << "Track " << it->second->id << endl;
					if (it->second->inactive)
						cout << " - Inactive for " << it->second->inactive << " frames" << endl;
					else
						cout << " - Associated with blobs " << it->second->label << endl;
					cout << " - Lifetime " << it->second->lifetime << endl;
					cout << " - Active " << it->second->active << endl;
					cout << " - Bounding box: (" << it->second->minx << ", " << it->second->miny << ") - (" << it->second->maxx << ", " << it->second->maxy << ")" << endl;
					cout << " - Centroid: (" << it->second->centroid.x << ", " << it->second->centroid.y << ")" << endl;
					cout << endl;
				}
			}
		}

		__CV_END__;
	}

	//void calculateMovement(cvb::CvTrack* track, short const length, float* degree, float* mag, float* acc, float* xv, float* yv) {
	void calculateMovement(cvb::CvTrack* track, short const length, float* degree, float* mag) {

		float sumx1 = 0.0;
		float sumy1 = 0.0;
		float avgx1 = 0.0;
		float avgy1 = 0.0;

		float sumx2 = 0.0;
		float sumy2 = 0.0;
		float avgx2 = 0.0;
		float avgy2 = 0.0;

		float vx = 0.0;
		float vy = 0.0;

		if (track->trajectory.size() >= length) {
			for (int j = 1; j <= length / 2; j++)
			{
				sumx1 += track->trajectory[track->trajectory.size() - j].x;
				sumy1 += track->trajectory[track->trajectory.size() - j].y;
			}
			avgx1 = 2 * sumx1 / length;
			avgy1 = 2 * sumy1 / length;

			for (int j = length / 2 + 1; j <= length; j++)
			{
				sumx2 += track->trajectory[track->trajectory.size() - j].x;
				sumy2 += track->trajectory[track->trajectory.size() - j].y;
			}
			avgx2 = 2 * sumx2 / length;
			avgy2 = 2 * sumy2 / length;
		}

		vx = avgx1 - avgx2;
		vy = avgy1 - avgy2;
		*degree = track->trajectory.size() >= length ? atan2(vy, vx) : 0.0;

		*mag = track->trajectory.size() >= length ? sqrt(vx * vx + vy * vy) : 1.0;
		//*acc = track->mags.size() > 2 ? (track->mags.back() - track->mags[track->mags.size() - 2]) : 0.0;

		//*xv = track->trajectory.size() >= length ? vx / track->trajectory : 0.0;
		//*yv = track->trajectory.size() >= length ? vy / track->trajectory : 0.0;
	}

	const CvTracks Copy(CvTracks tracks){
		CvTracks new_tracks;
		for (std::map<cvb::CvID, cvb::CvTrack*>::iterator it = tracks.begin(); it != tracks.end(); it++)
		{
			cvb::CvID id = (*it).first;
			cvb::CvTrack* track = new CvTrack; 
			track->id = id;
			track->label = (*it).second->label;
			track->minx = (*it).second->minx;
			track->miny = (*it).second->miny;
			track->maxx = (*it).second->maxx;
			track->maxy = (*it).second->maxy;
			track->centroid = (*it).second->centroid;
			track->lifetime = (*it).second->lifetime;
			track->active = (*it).second->active;
			track->inactive = (*it).second->inactive;
			track->pe_time = (*it).second->pe_time;
			track->pre_centroid = (*it).second->pre_centroid;
			new_tracks.insert(CvIDTrack(id, track));
		}
		return new_tracks;
	}
}
