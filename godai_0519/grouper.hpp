void face_group(
    std::vector<std::vector<cv::KeyPoint>>& dst,
    std::vector<cv::KeyPoint>& src,
	float error = 0.0f
    )
{
    for(size_t i=0; i<src.size(); ++i)
    {
        //距離全探索，短い順にソート
        std::vector<std::pair<size_t,double>> distances;
        for(size_t j=i+1; j<src.size(); ++j) distances.push_back(std::make_pair(j,calc_distance(src[i].pt, src[j].pt)));
        std::sort(
            distances.begin(), distances.end(),
            [](const std::pair<size_t,double>& lhs, const std::pair<size_t,double>& rhs)
            {
                return lhs.second < rhs.second;
            }
        );

        //近すぎもなく遠過ぎもないところをイテレータにのこす
        auto it = distances.cbegin();
        while(it != distances.cend())
        {
            const auto sum_size = src[i].size + src[it->first].size;
            if(sum_size < it->second && it->second < sum_size*(2.0+error)) break;
            else ++it;
        }
        if(it == distances.cend()) continue; //該当がなければスルー

        //2つの特徴点を詰める
        std::vector<cv::KeyPoint> group;
        group.push_back(src[i]);
        group.push_back(src[it->first]);
        src.erase(src.begin() + it->first);
        src.erase(src.begin() + i);

        //3つ目があるか探してみる
        const auto externally_point = get_externally_point(group.front().pt, group.back().pt);
        for(size_t j=0; j<src.size(); ++j)
        {
            const auto distance = calc_distance(externally_point, src[j].pt);
            const auto radius   = src[j].size;

            if(distance <= radius)
            {
                group.push_back(src[j]);
                src.erase(src.begin() + j);
                if(j < i) --i;

                //3つが見つかったんで5つ
                for(size_t k=0; k<src.size() && group.size()!=5; ++k)
                {
                    const auto distance_k_center = calc_distance(group[1].pt, src[k].pt);
                    const auto rabius_k_center   = group[1].size + src[k].size;
                    if(distance_k_center < rabius_k_center*2.5) continue;
                    
                    const line_segment<> k_zero_line(src[k].pt, group[0].pt);
                    for(size_t l=k+1; l<src.size() && group.size()!=5; ++l)
                    {
                        const line_segment<> l_two_line(src[l].pt, group[2].pt);

                        if(k_zero_line.is_parallel(l_two_line))
                        {
                            group.push_back(src[k]);
                            group.push_back(src[l]);
                            src.erase(src.begin() + l);
                            src.erase(src.begin() + k);

                            if(k <  i) --i;
                            if(l <  i) --i;
                        }
                    }
                }

                break;
            }
        }

        dst.push_back(std::move(group));
        --i;
    }

    std::cout << "Remain: " << src.size() << std::endl;

	if(error < 3.0) face_group(dst,src,error+0.5f);

    return;
}

void face_merge(
    std::vector<std::vector<cv::KeyPoint>>& dst,
    std::vector<std::vector<cv::KeyPoint>>& src
    )
{
    while(src.size() != 0)
    {
        std::vector<cv::KeyPoint> group;
        if(src[0].size() == 3)
        {
            for(size_t i=1; i<src.size(); ++i)
            {
                if(src[i].size() == 3)
                {
                    const line_segment<> line_base(src[0].front().pt, src[0].back().pt);
                    const line_segment<> line_other(src[i].front().pt, src[i].back().pt);
                    const float distance1 = calc_distance(src[0].front().pt, src[i].front().pt);
                    const float distance2 = calc_distance(src[0].back().pt , src[i].back().pt );
                    const float averadius = (src[0].front().size + src[0].back().size + src[i].front().size + src[i].back().size) / 4.0;

                    if(
                        line_base.is_parallel(line_other) && 
                        /*(averadius*3.0<distance1) && */(distance1<averadius*7.0) && 
                        /*(averadius*3.0<distance2) && */(distance2<averadius*7.0)
                        )
                    {
                        group.insert(group.end(), src[0].begin(), src[0].end());
                        group.insert(group.end(), src[i].begin(), src[i].end());
                        src.erase(src.begin() + i);
                        src.erase(src.begin());
                        break;
                    }
                }
                //else if(src[i].size() == 2)
                //{
                //    const line_segment<> line_one(src[0].front().pt, src[i].front().pt);
                //    const line_segment<> line_two(src[0].back().pt , src[i].back().pt );
                //    const cv::Point2f center((src[i].front().pt + src[i].back().pt)*0.50);
                //    if(calc_distance(center, src[0][1].pt) < src[0].at(1).size && line_one.is_parallel(line_two))
                //    {
                //        group.insert(group.end(), src[0].begin(), src[0].end());
                //        group.insert(group.end(), src[i].begin(), src[i].end());                        
                //        src.erase(src.begin() + i);
                //        src.erase(src.begin());
                //        break;
                //    }

                //}
            }
        }
        else if(src[0].size() == 2)
        {
            for(size_t i=1; i<src.size(); ++i)
            {
                if(src[i].size() != 2) continue;

                const line_segment<> line_base(src[0].front().pt, src[0].back().pt);
                const line_segment<> line_other(src[i].front().pt, src[i].back().pt);
                const float distance1 = calc_distance(src[0].front().pt, src[i].front().pt);
                const float distance2 = calc_distance(src[0].back().pt , src[i].back().pt );
                const float averadius = (src[0].front().size + src[0].back().size + src[i].front().size + src[i].back().size) / 4.0;

                if(
                    line_base.is_parallel(line_other) && 
                    /*(averadius*3.0<distance1) && */(distance1<averadius*7.0) && 
                    /*(averadius*3.0<distance2) && */(distance2<averadius*7.0)
                    )
                {
                    group.insert(group.end(), src[0].begin(), src[0].end());
                    group.insert(group.end(), src[i].begin(), src[i].end());
                    src.erase(src.begin() + i);
                    src.erase(src.begin());
                    
                    for(size_t j=0; j<src.size(); ++j)
                    {
                        if(src[j].size() != 2) continue;
                        const line_segment<> line_third(src[j].front().pt, src[j].back().pt);

                        //並行*3
                        {
                            const float distance3 = calc_distance(group[2].pt, src[j].front().pt);
                            const float distance4 = calc_distance(group[3].pt, src[j].back().pt);

                            if(
                                line_third.is_parallel(line_other) &&
                                (averadius*3.0<distance3) && (distance3<averadius*7.0) && 
                                (averadius*3.0<distance4) && (distance4<averadius*7.0)
                                )
                            {
                                group.insert(group.end(), src[j].begin(), src[j].end());
                                src.erase(src.begin() + j);
                                if(j < i) --i;
                                break;
                            }
                        }

                        //1つ直交
                        {                          
                            const line_segment<> line_02(group[0].pt, group[2].pt);
                            const float distance3 = calc_distance(group[0].pt, src[j].front().pt);
                            const float distance4 = calc_distance(group[2].pt, src[j].back().pt);
                            
                            if(
                                line_02.is_parallel(line_third) &&
                                (averadius*3.0<distance3) && (distance3<averadius*7.0) && 
                                (averadius*3.0<distance4) && (distance4<averadius*7.0)
                                )
                            {
                                group.insert(group.end(), src[j].begin(), src[j].end());
                                src.erase(src.begin() + j);
                                if(j < i) --i;
                                break;
                            }

                            
                            const line_segment<> line_13(group[1].pt, group[3].pt);
                            const float distance5 = calc_distance(group[1].pt, src[j].front().pt);
                            const float distance6 = calc_distance(group[3].pt, src[j].back().pt);
                            
                            if(
                                line_13.is_parallel(line_third) &&
                                (averadius*3.0<distance5) && (distance5<averadius*7.0) && 
                                (averadius*3.0<distance6) && (distance6<averadius*7.0)
                                )
                            {
                                group.insert(group.end(), src[j].begin(), src[j].end());
                                src.erase(src.begin() + j);
                                if(j < i) --i;
                                break;
                            }
                        }
                    }

                    break;
                }
            }
        }

        if(group.size() == 0)
        {
            group.insert(group.end(), src[0].begin(), src[0].end());
            src.erase(src.begin());
        }

        dst.push_back(std::move(group));
    }

    return;

}
