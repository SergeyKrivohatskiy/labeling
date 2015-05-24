#include "test_point_feature.h"

using namespace geom2;
namespace labeling
{
    const size_t TRACK_LEN = 200;

    test_point_feature::test_point_feature(const geom2::point_i &position,
                                           const geom2::point_d &speed,
                                           const geom2::size_i &field_size,
                                           bool is_fixed,
                                           double rotation)
        :
          speed(speed),
          position(position),
          field_size(field_size),
          exact_position(position.x, position.y),
          is_fixed(is_fixed),
          rotation(rotation),
          cur_rotation(0.0),
          offset_changed(false),
          label_offset(40, 40),
          exact_offset(static_cast<point_d>(label_offset)),
          track(TRACK_LEN, position)
    {
        label_size.h = 40;
        label_size.w = 100;
        prefered_positions.push_back(prefered_position(1.0, label_offset));
        prefered_positions.push_back(prefered_position(1.0, point_i{-40, 40}));
        prefered_positions.push_back(prefered_position(0.3, point_i{40, -40}));
    }

    test_point_feature::~test_point_feature()
    {}

    const point_i& test_point_feature::get_screen_pivot() const
    {
        return position;
    }
    const size_i& test_point_feature::get_label_size() const
    {
        return label_size;
    }

    const point_i& test_point_feature::get_label_offset() const
    {
        return label_offset;
    }

    void test_point_feature::set_label_offset(const point_i &new_offset)
    {
        offset_changed = true;
        label_offset = new_offset;
    }

    const test_point_feature::prefered_pos_list&
                        test_point_feature::get_prefered_positions() const
    {
        return rotated_prefered_positions;
    }

    bool test_point_feature::is_label_fixed() const
    {
        return is_fixed;
    }

    void test_point_feature::set_fixed(bool fixed)
    {
        is_fixed = fixed;
    }

    const std::deque<geom2::point_i>& test_point_feature::get_track() const
    {
        return track;
    }

    void test_point_feature::update_position()
    {
        track.pop_back();
        track.push_front(position);
        cur_rotation += rotation;
        point_d rotated_speed = rotate(speed, cur_rotation);
        if(offset_changed)
        {
            exact_offset = static_cast<point_d>(label_offset);
            offset_changed = false;
        } else {
            exact_offset = rotate(exact_offset, rotation);
            label_offset = static_cast<point_i>(exact_offset);
        }

        exact_position.x =
                fmod(exact_position.x + rotated_speed.x + field_size.w,
                     field_size.w);
        exact_position.y =
                fmod(exact_position.y + rotated_speed.y + field_size.h,
                     field_size.h);
        position = static_cast<point_i>(exact_position);
        rotated_prefered_positions.clear();
        for(const prefered_position &pos: prefered_positions)
        {
            point_i offset_to_center = static_cast<point_i>(label_size) / 2;
            point_i new_pos = static_cast<point_i>(
                        rotate(pos.second +
                               offset_to_center,
                               cur_rotation)) - offset_to_center;
            rotated_prefered_positions.push_back(prefered_position(pos.first,
                                                                   new_pos));
        }
    }

} // namespace labeling

