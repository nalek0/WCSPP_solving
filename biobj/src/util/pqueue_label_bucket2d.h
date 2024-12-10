#ifndef PQUEUE_LABEL_BUCKET2D_H
#define PQUEUE_LABEL_BUCKET2D_H

// A 2-level bucket-based priority queue for search_labels. 
// WITHOUT tie breaking using linked lists inside buckets
// First implementation pqueue_label_bucket: Fixed range buckets (needs f_max-f_min)
// Second implementation pqueue_label_bucket_cyclic: Cyclic buckets (needs bucket_size)
//
// 
//
// @author: sahmadi
// @created: 04/06/2021
// @updated: 28/01/2022
//

#include <cassert>
#include <iostream>


template <class LABEL>
class pqueue_label_bucket2d
{
    
	public:
    
        pqueue_label_bucket2d(size_t size=1, cost_t fmin=0, cost_t fmax=0)
            : max_cursor_(0), queuesize_(0), cursor_high_(0), cursor_low_(0), f_min(fmin)
        {
            high_level_ = new linkedlist<LABEL*>[1]();
            // bucket_width_ = size*10;
            bucket_width_ = (fmax - fmin)/1e6 + 1 ; // enforcing 1M buckets max
            low_level_ = new linkedlist<LABEL*>[bucket_width_]();
            initsize_ = (fmax - fmin)/bucket_width_ + 1;
            resize(initsize_);

        }

        ~pqueue_label_bucket2d()
        {
            delete [] high_level_;
            delete [] low_level_;
        }

        
        inline void
        range_update(cost_t fmin, cost_t fmax)
        {
            f_min = fmin;
            resize(fmax-fmin+1);
        }
        

        void
        clear()
        {
            queuesize_ = 0;
            max_cursor_ = initsize_;
            delete [] high_level_;
            delete [] low_level_;
            high_level_ = 0;
            cursor_high_ = 0;
        }

		// add a new element to the pqueue
        void 
        push (LABEL* lb)
        {
            size_t index = (lb->get_f_pri() - f_min)/bucket_width_;
            if(index == cursor_high_)
            (this->low_level_[(lb->get_f_pri() - f_min) % bucket_width_]).push_front(lb);
            else
            (this->high_level_[index]).push_front(lb);
            
            queuesize_++;
            return;
        }

		LABEL*
        pop()
        {
            if (queuesize_)
            {
                LABEL* ans = this->peek();
                (this->low_level_[cursor_low_]).pop_front();
                queuesize_--;
                return ans;
            }
            return 0;
        }

		
		inline LABEL*
		peek()
		{
            if (queuesize_)
            {

                if (bucket_width_ == 1)
                {
                    if (!low_level_[cursor_low_].size())
                    {    
                        while (cursor_high_ <= max_cursor_ && !high_level_[cursor_high_].size())
                        {cursor_high_++;}
                        this->low_level_[cursor_low_] = high_level_[cursor_high_];
                        high_level_[cursor_high_].clear();
                    }
                    
                }
                else
                {
                    while(cursor_low_ < bucket_width_ && !low_level_[cursor_low_].size())
                    {cursor_low_++;}
                    
                    if(cursor_low_ == bucket_width_)
                    {
                        while (cursor_high_ <= max_cursor_ && !high_level_[cursor_high_].size())
                        {cursor_high_++;}
                    
                        while (high_level_[cursor_high_].size() > 0)
                        {
                            LABEL* lb = high_level_[cursor_high_].front();
                            high_level_[cursor_high_].pop_front();
                            size_t index = (lb->get_f_pri() - f_min) % bucket_width_;
                            (this->low_level_[index]).push_back(lb);
                        }
                    }
                    cursor_low_ = 0;

                    while(cursor_low_ < bucket_width_ && !low_level_[cursor_low_].size())
                    {cursor_low_++;}
                }
                
                return this->low_level_[cursor_low_].front();
            }
            else
            {return 0;}
        }

        // inline cost_t 
		// peek_f(void)
		// {
        //     LABEL* ans = this->peek();
            
        //     if(ans)
		// 	{
        //         return this->cursor_+this->f_min;
        //     }
        //     // std::cerr<<"Error: Accessing out of Bucket list."<<std::endl;
        //     return COST_MAX;
        // }

        // inline cost_t 
		// current_f(void)
		// {
        //     return this->cursor_+this->f_min;
        // }

		inline unsigned int
		size() const
		{
			return queuesize_;
		}

        void
        resize(size_t newsize)
        {
            if(newsize - 1 <= max_cursor_)
            {
                return;
            }

            linkedlist<LABEL*>* tmp_ = new linkedlist<LABEL*>[newsize]();

            for(size_t i=0; i <= max_cursor_; i++)
	        {
                tmp_[i] = high_level_[i];
            }
            
            delete [] high_level_;
            high_level_ = tmp_;
            max_cursor_ = newsize-1;
            return;
        }

        inline size_t
        get_cmp()
        {
            return cursor_high_;
        }

        void 
        pull_up(LABEL* lb, LABEL* lb_old)
        {
            lb_old->set_f_pri(0);
            this->push(lb);
            return;
        
        }


        void 
        pull_down(LABEL* lb)
        {
            return;
        }
        
        // inline void
        // size_update(cost_t fmax)
        // {
        //     uint new_size = fmax - f_min +1;
        //     while(max_cursor_+1 != new_size)
        //     {
        //         while(this->elts_head_[max_cursor_])
        //             {
        //                 L* tmp = this->elts_head_[max_cursor_];
        //                 this->elts_head_[max_cursor_]=tmp->next_;
        //                 label_pool->save_label(tmp);
        //                 queuesize_--;
        //             } 
        //         // queuesize_ = queuesize_ - counter[max_cursor_];
        //         max_cursor_--;
        //     }
            
            
        // }

        size_t
		mem()
		{
			return (max_cursor_ + bucket_width_)*sizeof (LABEL*)
				+ sizeof(*this);
		}

    private:
		size_t initsize_;
		size_t max_cursor_;
		unsigned int queuesize_;
		size_t cursor_high_;
        size_t cursor_low_;
        cost_t f_min;
        size_t bucket_width_;
        linkedlist<LABEL*>* high_level_;
        linkedlist<LABEL*>* low_level_;
    
};

typedef pqueue_label_bucket2d<search_label> pqueue_label_bucket2d_min;
typedef pqueue_label_bucket2d<search_label_light> pqueue_label_light_bucket2d_min;

#endif