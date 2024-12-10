#ifndef SEARCH_LABEL_H
#define SEARCH_LABEL_H

class search_label
{
public:
	search_label()
	{
		f_pri_ = 0;
		f_sec_ = 0;
		id_ = SN_ID_MAX;
		next_ = 0;
	}

	~search_label()
	{
	}

	search_label(cost_t f_pri, cost_t f_sec, sn_id_t id)
	{
		f_pri_ = f_pri;
		f_sec_ = f_sec;
		id_ = id;
	}
	search_label(cost_t f_pri, cost_t f_sec, sn_id_t id, search_label *parent)
	{
		f_pri_ = f_pri;
		f_sec_ = f_sec;
		id_ = id;
		next_ = parent;
	}
	search_label(cost_t f_pri, cost_t f_sec, sn_id_t id, edge_cap_t incoming_edge, path_arr_size index)
	{
		f_pri_ = f_pri;
		f_sec_ = f_sec;
		id_ = id;
		incoming_edge_ = incoming_edge;
		path_id_ = index;
	}

	inline void
	set_f_pri(cost_t f_pri)
	{
		f_pri_ = f_pri;
	}

	inline void
	set_id(sn_id_t id)
	{
		id_ = id;
	}

	inline cost_t
	get_f_pri() const
	{
		return f_pri_;
	}

	inline cost_t
	get_f_sec() const
	{
		return f_sec_;
	}

	inline sn_id_t
	get_id() const
	{
		return id_;
	}

	inline edge_cap_t
	get_incoming_edge()
	{
		return incoming_edge_;
	}

	inline path_arr_size
	get_path_id()
	{
		return path_id_;
	}

	inline search_label *
	get_parent()
	{
		return NULL;
	}

	inline uint32_t
	get_priority()
	{
		return 0;
	}

	inline void
	set_priority(uint32_t priority)
	{
		
	}

	inline search_label *
	get_next()
	{
		return next_;
	}

	inline void
	set_next(search_label *next)
	{
		next_ = next;
	}

	size_t
	mem()
	{
		return sizeof(*this);
	}

private:
	cost_t f_pri_, f_sec_;
	sn_id_t id_;
	search_label *next_ = 0; // for linked list
	edge_cap_t incoming_edge_;
	path_arr_size path_id_;
};
///////////////////////////////////////
//// LIGHT SEARCH LABEL
class search_label_light
{
public:
	search_label_light()
	{
		f_pri_ = 0;
		f_sec_ = 0;
		id_ = SN_ID_MAX;
		next_ = 0;
	}

	~search_label_light()
	{
	}

	search_label_light(cost_t f_pri, cost_t f_sec, sn_id_t id)
	{
		f_pri_ = f_pri;
		f_sec_ = f_sec;
		id_ = id;
	}
	search_label_light(cost_t f_pri, cost_t f_sec, sn_id_t id, search_label_light *parent)
	{
		f_pri_ = f_pri;
		f_sec_ = f_sec;
		id_ = id;
		next_ = parent;
	}
	search_label_light(cost_t f_pri, cost_t f_sec, sn_id_t id, edge_cap_t incoming_edge, path_arr_size index)
	{
		f_pri_ = f_pri;
		f_sec_ = f_sec;
		id_ = id;
	}

	inline void
	set_f_pri(cost_t f_pri)
	{
		f_pri_ = f_pri;
	}

	inline void
	set_id(sn_id_t id)
	{
		id_ = id;
	}

	inline cost_t
	get_f_pri() const
	{
		return f_pri_;
	}

	inline cost_t
	get_f_sec() const
	{
		return f_sec_;
	}

	inline sn_id_t
	get_id() const
	{
		return id_;
	}

	inline edge_cap_t
	get_incoming_edge()
	{
		return 0;
	}

	inline path_arr_size
	get_path_id()
	{
		return 0;
	}

	inline search_label_light *
	get_parent()
	{
		return NULL;
	}

	inline uint32_t
	get_priority()
	{
		return 0;
	}

	inline void
	set_priority(uint32_t priority)
	{
		
	}

	inline search_label_light *
	get_next()
	{
		return next_;
	}

	inline void
	set_next(search_label_light *next)
	{
		next_ = next;
	}

	size_t
	mem()
	{
		return sizeof(*this);
	}

private:
	cost_t f_pri_, f_sec_;
	sn_id_t id_;
	search_label_light *next_ = 0; // for linked list
};
///////////////////////////////////////
//// SEARCH LABEL WITH PRIORITY
class search_label_pri
{
public:
	search_label_pri()
	{
		f_pri_ = 0;
		f_sec_ = 0;
		id_ = SN_ID_MAX;
		next_ = 0;
	}

	~search_label_pri()
	{
	}

	search_label_pri(cost_t f_pri, cost_t f_sec, sn_id_t id)
	{
		f_pri_ = f_pri;
		f_sec_ = f_sec;
		id_ = id;
	}
	search_label_pri(cost_t f_pri, cost_t f_sec, sn_id_t id, search_label_pri *parent)
	{
		f_pri_ = f_pri;
		f_sec_ = f_sec;
		id_ = id;
		next_ = parent;
	}
	search_label_pri(cost_t f_pri, cost_t f_sec, sn_id_t id, edge_cap_t incoming_edge, path_arr_size index)
	{
		f_pri_ = f_pri;
		f_sec_ = f_sec;
		id_ = id;
		incoming_edge_ = incoming_edge;
		path_id_ = index;
	}

	inline void
	set_f_pri(cost_t f_pri)
	{
		f_pri_ = f_pri;
	}

	inline void
	set_id(sn_id_t id)
	{
		id_ = id;
	}

	inline cost_t
	get_f_pri() const
	{
		return f_pri_;
	}

	inline cost_t
	get_f_sec() const
	{
		return f_sec_;
	}

	inline sn_id_t
	get_id() const
	{
		return id_;
	}

	inline edge_cap_t
	get_incoming_edge()
	{
		return incoming_edge_;
	}

	inline path_arr_size
	get_path_id()
	{
		return path_id_;
	}

	inline search_label_pri *
	get_parent()
	{
		return NULL;
	}

	inline uint32_t
	get_priority()
	{
		return priority_;
	}

	inline void
	set_priority(uint32_t priority)
	{
		priority_ = priority;
	}

	inline search_label_pri *
	get_next()
	{
		return next_;
	}

	inline void
	set_next(search_label_pri *next)
	{
		next_ = next;
	}

	size_t
	mem()
	{
		return sizeof(*this);
	}

private:
	cost_t f_pri_, f_sec_;
	sn_id_t id_;
	search_label_pri *next_ = 0; // for linked list
	edge_cap_t incoming_edge_;
	path_arr_size path_id_;
	uint32_t priority_;
};
///////////////////////////////////////
//// LIGHT SEARCH LABEL WITH PRIORITY

class search_label_light_pri
{
public:
	search_label_light_pri()
	{
		f_pri_ = 0;
		f_sec_ = 0;
		id_ = SN_ID_MAX;
		next_ = 0;
	}

	~search_label_light_pri()
	{
	}

	search_label_light_pri(cost_t f_pri, cost_t f_sec, sn_id_t id)
	{
		f_pri_ = f_pri;
		f_sec_ = f_sec;
		id_ = id;
	}
	search_label_light_pri(cost_t f_pri, cost_t f_sec, sn_id_t id, search_label_light_pri *parent)
	{
		f_pri_ = f_pri;
		f_sec_ = f_sec;
		id_ = id;
		next_ = parent;
	}
	search_label_light_pri(cost_t f_pri, cost_t f_sec, sn_id_t id, edge_cap_t incoming_edge, path_arr_size index)
	{
		f_pri_ = f_pri;
		f_sec_ = f_sec;
		id_ = id;
	}

	inline void
	set_f_pri(cost_t f_pri)
	{
		f_pri_ = f_pri;
	}

	inline void
	set_id(sn_id_t id)
	{
		id_ = id;
	}

	inline cost_t
	get_f_pri() const
	{
		return f_pri_;
	}

	inline cost_t
	get_f_sec() const
	{
		return f_sec_;
	}

	inline sn_id_t
	get_id() const
	{
		return id_;
	}

	inline edge_cap_t
	get_incoming_edge()
	{
		return 0;
	}

	inline path_arr_size
	get_path_id()
	{
		return 0;
	}

	inline search_label_light_pri *
	get_parent()
	{
		return NULL;
	}

	inline uint32_t
	get_priority()
	{
		return priority_;
	}

	inline void
	set_priority(uint32_t priority)
	{
		priority_ = priority;
	}

	inline search_label_light_pri *
	get_next()
	{
		return next_;
	}

	inline void
	set_next(search_label_light_pri *next)
	{
		next_ = next;
	}

	size_t
	mem()
	{
		return sizeof(*this);
	}

private:
	cost_t f_pri_, f_sec_;
	sn_id_t id_;
	search_label_light_pri *next_ = 0; // for linked list
	uint32_t priority_;
};

#endif
