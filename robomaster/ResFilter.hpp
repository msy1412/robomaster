
#ifndef ResFilter_H
#define ResFilter_H
#include <list>

class RuneResFilter {
public:
	RuneResFilter(int _filter_size = 5, int _shoot_time_gap = 100)
		: filter_size(_filter_size + 1), shoot_time_gap(_shoot_time_gap), last_shoot_idx(-1), last_shoot_time(0){}
	bool setRecord(int record);
    void clear(){
        last_shoot_idx = -1;
        last_shoot_time = 0;
        history.clear();
    }
    /**
     * @brief getResult
     * @return true, if latest record has the most votes and the num of votes large than half of the size of history record
     */
	bool getResult();
	bool isShootable(double anglex, double angley, double z, int cell_idx);

private:
	std::list<int> history;
	int filter_size;
	int shoot_time_gap;
	int last_shoot_idx;
	int last_shoot_time;
};

class ArmorFilter {
public:
	ArmorFilter(int _filter_size = 5)
		: filter_size(_filter_size) {}

    void clear(){
        history.clear();
    }

    bool getResult(bool is_small){
        if (history.size() < filter_size){
            history.push_back(is_small);
        }
        else {
            history.push_back(is_small);
            history.pop_front();
        }

        int vote_cnt[2] = {0}; // cnt[0]--Armor cnt[1]--small_Armor

        for (std::list<bool>::const_iterator it = history.begin(); it != history.end(); ++it){
            *it == 0 ? ++vote_cnt[0] : ++vote_cnt[1];
        }

        if (vote_cnt[0] == vote_cnt[1])
            return is_small;
        return vote_cnt[0] > vote_cnt[1] ? 0 : 1;
    }

private:
	std::list<bool> history;
	int filter_size;
};

#endif