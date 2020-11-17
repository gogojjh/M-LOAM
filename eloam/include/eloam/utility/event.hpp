// This file is part of dvs-panotracking.
//
// Copyright (C) 2017 Christian Reinbacher <reinbacher at icg dot tugraz dot at>
// Institute for Computer Graphics and Vision, Graz University of Technology
// https://www.tugraz.at/institute/icg/teams/team-pock/
//
// dvs-panotracking is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// dvs-panotracking is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef EVENT_HPP
#define EVENT_HPP

namespace eloam
{
class Event
{

public:
    Event(int x, int y, double time, bool polarity)
    {
        x_ = x;
        y_ = y;
        time_ = time;
        polarity_ = polarity ? 1 : 0;
    }

    int x_;
    int y_;
    float x_undist_;
    float y_undist_;
    double time_;
    float polarity_;
};
   
} // namespace eloam

#endif // EVENT_HPP
