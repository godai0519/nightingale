
template<typename Ty>
inline double point_to_far(const cv::Point_<Ty>& point)
{
    return hypot(point.x, point.y);
}
template<typename Type>
inline double calc_distance(const cv::Point_<Type> &p1,const cv::Point_<Type> &p2)
{
    return point_to_far(p1 - p2);
}
std::vector<cv::Point2f> rect_to_points(const cv::RotatedRect& rect)
{
    cv::Point2f points[4];
    rect.points(points);

    std::vector<cv::Point2f> result;
    for(int i=0; i<4; ++i) result.push_back(points[i]);
    return result;
}

void delete_overlap(const std::vector<std::vector<cv::Point>>& red, std::vector<cv::KeyPoint>& black)
{
	for(auto red_it = red.cbegin(); red_it != red.cend(); ++red_it)
	{
		for(auto black_it = black.begin(); black_it != black.end();)
		{
			if(cv::pointPolygonTest(*red_it, black_it->pt, false) >= 0) //内側
			{
				black_it = black.erase(black_it);
			}
			else ++black_it;				
		}
	}
	return;
}

void delete_overlap(std::vector<std::pair<cv::Point2f,float>>& faces)
{
	std::sort(faces.begin(), faces.end(),
		[](const std::pair<cv::Point2f,float>& lhs, const std::pair<cv::Point2f,float>& rhs)
		{
			return lhs.second < rhs.second;
		});

	for(size_t i = 0; i<faces.size();++i)
	{
		for(size_t j=i+1; j<faces.size();)
		{
			if(calc_distance(faces[i].first, faces[j].first) < faces[j].second)
			{
				faces.erase(faces.begin() + j);
			}
			else ++j;
		}
	}
}

template<class T = float>
class line_segment{
    const cv::Point_<T> orient_vector_;
    const cv::Point_<T> fixed_point1_;
    const cv::Point_<T> fixed_point2_;
    
public:
    line_segment(const T& x1,const T& y1,const T& x2,const T& y2)
    : orient_vector_(x2-x1,y2-y1),
        fixed_point1_(x1,y1),
        fixed_point2_(x2,y2)
    {
    }
    line_segment(const cv::Point_<T>& p1,const cv::Point_<T>& p2)
    : orient_vector_(p2.x-p1.x,p2.y-p1.y),
        fixed_point1_(p1),
        fixed_point2_(p2)
    {
    }
    
    bool is_parallel(const line_segment& other,const T& tolerance_angle = PI/36) const
    {
        const double x1 = this->orient_vector_.x, y1 = this->orient_vector_.y;
        const double x2 = other.orient_vector_.x, y2 = other.orient_vector_.y;
        const double angle = acos((x1*x2 + y1*y2) / (hypot(x1,y1)*hypot(x2,y2)));

        return (-tolerance_angle <= angle && angle <= tolerance_angle);
    }
    bool is_normal(const line_segment& other,const T& tolerance_angle = PI/36) const
    {
        const double x1 = this->orient_vector_.x, y1 = this->orient_vector_.y;
        const double x2 = other.orient_vector_.x, y2 = other.orient_vector_.y;
        const double angle = acos((x1*x2 + y1*y2) / (hypot(x1,y1)*hypot(x2,y2)));
    
        const double right_angle = PI/2;
        return (right_angle-tolerance_angle <= angle && angle <= right_angle+tolerance_angle);
    }

    T distance(const cv::Point_<T>& point) //適当
    {
        const cv::Point_<T> vec_ab(fixed_point2_ - fixed_point1_);
        const cv::Point_<T> vec_ac(point - fixed_point1_);
        if(vec_ab.dot(vec_ac) < 0.0) return hypot(vec_ac.x ,vec_ac.y);

        const cv::Point_<T> vec_ba(fixed_point1_ - fixed_point2_);
        const cv::Point_<T> vec_bc(point - fixed_point2_);
        if(vec_ba.dot(vec_bc) < 0.0) return hypot(vec_bc.x, vec_bc.y);

        return std::abs(vec_ab.cross(vec_ac)) / hypot(vec_ab.x, vec_ab.y);
    }

    bool operator|| (const line_segment& other) const
    {
        return is_parallel(other);
    }
    bool operator+ (const line_segment& other) const
    {
        return is_normal(other);
    }
};

template<typename Ty>
inline cv::Point_<Ty> get_externally_point(
    const cv::Point_<Ty>& i,
    const cv::Point_<Ty>& j
    )
{
    return cv::Point_<Ty>(2.0*j - i);
}

//四隅でなく，上下左右．適当
void circle_to_four_point(std::vector<cv::Point2f>& dst, const cv::Point2f& center, const float radius)
{
    dst.push_back(cv::Point2f(center.x + radius, center.y         ));
    dst.push_back(cv::Point2f(center.x - radius, center.y         ));
    dst.push_back(cv::Point2f(center.x         , center.y + radius));
    dst.push_back(cv::Point2f(center.x         , center.y - radius));

    return;
}

void erase_big_eye(std::vector<cv::KeyPoint>& eyes, int bigest_size)
{
	for(auto it = eyes.begin(); it != eyes.end();)
	{
		if(it->size > bigest_size)
		{
			it = eyes.erase(it);
		}
		else ++it;
	}

	return;
}

void erase_small_eye(std::vector<cv::KeyPoint>& eyes, int smallest_size)
{
	for(auto it = eyes.begin(); it != eyes.end();)
	{
		if(it->size < smallest_size)
		{
			it = eyes.erase(it);
		}
		else ++it;
	}

	return;
}

void erase_big_faces(std::vector<std::pair<cv::Point2f,float>>& faces, float biggest = 64)
{
	for(auto it = faces.begin(); it != faces.end(); )
	{
		if(it->second > biggest)
		{
			it = faces.erase(it);
		}
		else ++it;
	}
	return;
}

