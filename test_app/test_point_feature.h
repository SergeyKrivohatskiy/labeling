#ifndef TEST_POINT_FEATURE_H
#define TEST_POINT_FEATURE_H

#include "labeling/screen_point_feature.h"

namespace labeling
{
    class test_point_feature : public screen_point_feature
    {
    public:
        test_point_feature(const geom2::point_i &position,
                           const geom2::point_d &speed,
                           const geom2::size_i &field_size,
                           bool is_fixed,
                           double rotation = 0);
        ~test_point_feature();

        const geom2::point_i& get_screen_pivot() const;
        const geom2::size_i& get_label_size() const;

        const geom2::point_i& get_label_offset() const;
        void set_label_offset(const geom2::point_i&);

        bool is_label_fixed() const;
        void set_fixed(bool fixed);

        const prefered_pos_list& get_prefered_positions() const;

        void update_position();
    private:
        geom2::point_i position;
        geom2::point_d exact_position;
        geom2::point_d speed;
        geom2::size_i field_size;
        geom2::size_i label_size;
        geom2::point_i label_offset;
        prefered_pos_list prefered_positions;
        bool is_fixed;
    };
} // namespace labeling

#endif // TEST_POINT_FEATURE_H
