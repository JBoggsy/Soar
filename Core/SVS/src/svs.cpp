#include <cassert>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <utility>
#include <algorithm>
#include <fstream>
// SVS core includes
#include "svs.h"
#include "sgnode.h"
#include "scene.h"
#include "common.h"
#include "symbol.h"
// SVS interface includes
#include "command.h"
#include "soar_interface.h"
#include "filter_table.h"
#include "command_table.h"
// SVS visualization includes
#include "drawer.h"
// SVS vision includes
#include "image.h"
#include "exact_visual_concept_descriptor.h"
#include "visual_long_term_memory.h"
#include "visual_working_memory.h"

#include "symbol.h"

typedef std::map<std::string, command*>::iterator cmd_iter;

svs_interface* make_svs(agent* a)
{
    return new svs(a);
}


sgwme::sgwme(soar_interface* si, Symbol* ident, sgwme* parent, sgnode* node)
    : soarint(si), id(ident), parent(parent), node(node)
{
    node->listen(this);
    id_wme = soarint->make_wme(id, si->get_common_syms().id, node->get_id());

    if (node->is_group())
    {
        group_node* g = node->as_group();
        for (size_t i = 0; i < g->num_children(); ++i)
        {
            add_child(g->get_child(i));
        }
    }

    const tag_map& node_tags = node->get_all_tags();
    tag_map::const_iterator ti;
    for (ti = node_tags.begin(); ti != node_tags.end(); ti++)
    {
        set_tag(ti->first, ti->second);
    }
}

sgwme::~sgwme()
{
    std::map<sgwme*, wme*>::iterator i;

    if (node)
    {
        node->unlisten(this);
    }
    soarint->remove_wme(id_wme);

    std::map<std::string, wme*>::iterator ti;
    for (ti = tags.begin(); ti != tags.end(); ti++)
    {
        soarint->remove_wme(ti->second);
    }

    for (i = childs.begin(); i != childs.end(); ++i)
    {
        i->first->parent = NULL;
        delete i->first;
        soarint->remove_wme(i->second);
    }
    if (parent)
    {
        std::map<sgwme*, wme*>::iterator ci = parent->childs.find(this);
        assert(ci != parent->childs.end());
        soarint->remove_wme(ci->second);
        parent->childs.erase(ci);
    }
}

void sgwme::node_update(sgnode* n, sgnode::change_type t, const std::string& update_info)
{
    int added_child = 0;
    group_node* g;
    switch (t)
    {
        case sgnode::CHILD_ADDED:
            if (parse_int(update_info, added_child))
            {
                g = node->as_group();
                add_child(g->get_child(added_child));
            }
            break;
        case sgnode::DELETED:
            node = NULL;
            delete this;
            break;
        case sgnode::TAG_CHANGED:
            update_tag(update_info);
            break;
        case sgnode::TAG_DELETED:
            delete_tag(update_info);
            break;
        default:
            break;
    };
}

void sgwme::add_child(sgnode* c)
{
    char letter;
    std::string cid = c->get_id();
    sgwme* child;

    if (cid.size() == 0 || !isalpha(cid[0]))
    {
        letter = 'n';
    }
    else
    {
        letter = cid[0];
    }
    wme* cid_wme = soarint->make_id_wme(id, "child");

    child = new sgwme(soarint, soarint->get_wme_val(cid_wme), this, c);
    childs[child] = cid_wme;
}

void sgwme::set_tag(const std::string& tag_name, const std::string& tag_value)
{
    Symbol* rootID = id;
    std::string att = tag_name;

    wme* value_wme;
    if (map_get(tags, tag_name, value_wme))
    {
        soarint->remove_wme(value_wme);
    }
    tags[tag_name] = soarint->make_wme(rootID, att, tag_value);
}

void sgwme::update_tag(const std::string& tag_name)
{
    std::string tag_value;
    if (node->get_tag(tag_name, tag_value))
    {
        set_tag(tag_name, tag_value);
    }
}

void sgwme::delete_tag(const std::string& tag_name)
{
    wme* value_wme;
    if (map_get(tags, tag_name, value_wme))
    {
        soarint->remove_wme(value_wme);
        tags.erase(tag_name);
    }
}


/////////////////////
// SVS_STATE CLASS //
/////////////////////

svs_state::svs_state(svs* svsp, Symbol* state, soar_interface* soar, scene* scn)
    : svsp(svsp), parent(NULL), level(0), scn(scn), si(soar), state(state),
      scene_num(-1), scene_num_wme(NULL), scene_link(NULL)
{
    assert(state->is_top_state());
    state->get_id_name(name);
    init();
}

svs_state::svs_state(Symbol* state, svs_state* parent)
    : svsp(parent->svsp), parent(parent), level(parent->level + 1), scn(NULL),
      si(parent->si), state(state), scene_num(-1), scene_num_wme(NULL), scene_link(NULL)
{
    assert(state->get_parent_state() == parent->state);
    init();
}

svs_state::~svs_state()
{
    command_set_it i, iend;

    for (i = curr_cmds.begin(), iend = curr_cmds.end(); i != iend; ++i)
    {
        delete i->cmd;
    }

    if (scn)
    {
        svsp->get_drawer()->delete_scene(scn->get_name());
        delete scn; // results in root being deleted also
    }
}

void svs_state::init()
{
    common_syms& cs = si->get_common_syms();

    state->get_id_name(name);
    svs_link = si->get_wme_val(si->make_svs_wme(state));
    cmd_link = si->get_wme_val(si->make_id_wme(svs_link, cs.cmd));
    scene_link = si->get_wme_val(si->make_id_wme(svs_link, cs.scene));
    vwm_link = si->get_wme_val(si->make_id_wme(svs_link, cs.vwm));

    if (!scn)
    {
        if (parent)
        {
            scn = parent->scn->clone(name);
            vwm = parent->vwm->clone(vwm_link);
        }
        else
        {
            // top state
            scn = new scene(name, svsp);
            scn->set_draw(true);
            vwm = new visual_working_memory(svsp, si, vwm_link);
        }
    }
    scn->refresh_draw();
    root = new sgwme(si, scene_link, (sgwme*) NULL, scn->get_root());
}

void svs_state::update_scene_num()
{
    long curr;
    if (scene_num_wme)
    {
        if (!get_symbol_value(si->get_wme_val(scene_num_wme), curr))
        {
            exit(1);
        }
        if (curr == scene_num)
        {
            return;
        }
        si->remove_wme(scene_num_wme);
    }
    if (scene_num >= 0)
    {
        scene_num_wme = si->make_wme(svs_link, "scene-num", scene_num);
    }
}

void svs_state::update_cmd_results(int command_type)
{
    command_set_it i;
    for (i = curr_cmds.begin(); i != curr_cmds.end(); ++i)
    {
        if (i->cmd->command_type() == command_type)
        {
            i->cmd->update();
        }
    }
}

void svs_state::process_cmds()
{
    // Retrieve all the child WMEs of the command link and put them into a vector
    wme_vector all;
    wme_vector::iterator all_it;
    si->get_child_wmes(cmd_link, all);

    command_set live_commands;
    for (all_it = all.begin(); all_it != all.end(); all_it++)
    {
        // Convert wme val to std::string
        Symbol* idSym = si->get_wme_val(*all_it);
        std::string cmdId;
        ;
        if (!idSym->get_id_name(cmdId))
        {
            // Not an identifier, continue;
            continue;
        }

        live_commands.insert(command_entry(cmdId, 0, *all_it));
    }
    // Do a diff on the curr_cmds list and the live_commands
    //   to find which have been added and which have been removed
    std::vector<command_set_it> old_commands, new_commands;
    command_set_it live_it = live_commands.begin();
    command_set_it curr_it = curr_cmds.begin();
    while (live_it != live_commands.end() || curr_it != curr_cmds.end())
    {
        if (live_it == live_commands.end())
        {
            old_commands.push_back(curr_it);
            curr_it++;
        }
        else if (curr_it == curr_cmds.end())
        {
            new_commands.push_back(live_it);
            live_it++;
        }
        else if (curr_it->id == live_it->id)
        {
            curr_it++;
            live_it++;
        }
        else if (curr_it->id.compare(live_it->id) < 0)
        {
            old_commands.push_back(curr_it);
            curr_it++;
        }
        else
        {
            new_commands.push_back(live_it);
            live_it++;
        }
    }

    // Delete the command
    std::vector<command_set_it>::iterator old_it;
    for (old_it = old_commands.begin(); old_it != old_commands.end(); old_it++)
    {
        command_set_it old_cmd = *old_it;
        delete old_cmd->cmd;
        curr_cmds.erase(old_cmd);
    }

    // Add the new commands
    std::vector<command_set_it>::iterator new_it;
    for (new_it = new_commands.begin(); new_it != new_commands.end(); new_it++)
    {
        command_set_it new_cmd = *new_it;
        command* c = get_command_table().make_command(this, new_cmd->cmd_wme);
        if (c)
        {
            curr_cmds.insert(command_entry(new_cmd->id, c, 0));
            svs::mark_filter_dirty_bit();
        }
        else
        {
            std::string attr;
            get_symbol_value(si->get_wme_attr(new_cmd->cmd_wme), attr);
        }
    }
}

void svs_state::clear_scene()
{
    scn->clear();
}

void svs_state::proxy_get_children(std::map<std::string, cliproxy*>& c)
{
    c["scene"]        = scn;
    c["vwm"]          = vwm;
}

void svs_state::disown_scene()
{
    delete root;
    scn = NULL;
}

///////////////
// SVS CLASS //
///////////////

svs::svs(agent* a)
    : scn_cache(NULL), enabled(false)
{
    si = new soar_interface(a);
    draw = new drawer();

#ifdef ENABLE_OPENCV
    v_mem_opencv = new exact_opencv_mem(this);
#endif

#ifdef ENABLE_ROS
    ros_interface::init_ros();
    ri = new ros_interface(this);
    ri->start_ros();
#endif
}

bool svs::filter_dirty_bit = true;

svs::~svs()
{
    for (size_t i = 0, iend = state_stack.size(); i < iend; ++i)
    {
        delete state_stack[i];
    }
    if (scn_cache)
    {
        delete scn_cache;
    }

    delete si;
    delete draw;
}

void svs::state_creation_callback(Symbol* state)
{
    std::string type, msg;
    svs_state* s;

    // The first SVS state gets the VIB manager
    if (state_stack.empty())
    {
        common_syms& cs = si->get_common_syms();
        if (scn_cache)
        {
            scn_cache->verify_listeners();
        }
        s = new svs_state(this, state, si, scn_cache);
        scn_cache = NULL;

        vib_link = si->get_wme_val(si->make_id_wme(s->get_svs_link(), cs.vib));
        vib_manager = new visual_input_buffer_manager(si, vib_link);
    }
    else
    {
        s = new svs_state(state, state_stack.back());
    }

    state_stack.push_back(s);
}

void svs::state_deletion_callback(Symbol* state)
{
    svs_state* s;
    s = state_stack.back();
    assert(state == s->get_state());
    if (state_stack.size() == 1)
    {
        // removing top state, save scene for reinit
        scn_cache = s->get_scene();
        s->disown_scene();
    }

    delete s;
    state_stack.pop_back();
}

void svs::proc_input(svs_state* s)
{
#ifdef ENABLE_ROS
    boost::lock_guard<boost::mutex> guard(input_mtx);
#endif

    for (size_t i = 0; i < env_inputs.size(); ++i)
    {
        strip(env_inputs[i], " \t");
        s->get_scene()->parse_sgel(env_inputs[i]);
    }
    if (env_inputs.size() > 0)
    {
        svs::mark_filter_dirty_bit();
    }
    env_inputs.clear();
}

/**
 * This method is called by run_soar.cpp:833 at the beginning of every output phase.
 */
void svs::output_callback()
{
    if (!enabled)
    {
        return;
    }
    std::vector<svs_state*>::iterator i;
    std::string sgel;

    for (i = state_stack.begin(); i != state_stack.end(); ++i)
    {
        (**i).process_cmds();
    }
    //for (i = state_stack.begin(); i != state_stack.end(); ++i)
    //{
    //    (**i).update_cmd_results(true);
    //}

}

void svs::input_callback()
{
    if (!enabled)
    {
        return;
    }
    svs_state* topstate = state_stack.front();
    proc_input(topstate);

    std::vector<svs_state*>::iterator i;

    for (i = state_stack.begin(); i != state_stack.end(); ++i)
    {
        (**i).update_cmd_results(SVS_WRITE_COMMAND);
    }

    for (i = state_stack.begin(); i != state_stack.end(); ++i)
    {
        (**i).update_cmd_results(SVS_READ_COMMAND);
    }

    std::vector<std::string> vib_ids = get_vib_manager()->get_vib_ids();


    for (std::string vib_id : vib_ids){
        for (svs_state* state : state_stack) {
            state->get_vwm()->evaluate_vib_nodes(vib_id);
        }
    }

    svs::filter_dirty_bit = false;
}

#ifdef ENABLE_ROS
void svs::image_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& new_img)
{
    throw std::bad_function_call();
}
#endif

#ifdef ENABLE_OPENCV
void svs::image_callback(const cv::Mat& new_img)
{
    throw std::bad_function_call();
}
#endif

/*
 This is a naive implementation. If this method is called concurrently
 with proc_input, the env_inputs std::vector will probably become
 inconsistent. This eventually needs to be replaced by a thread-safe FIFO.
 XXX: Lizzie used a mutex to lock the the env_inputs structure since
      ROS will update from a separate thread.
*/
void svs::add_input(const std::string& in)
{
#ifdef ENABLE_ROS
    boost::lock_guard<boost::mutex> guard(input_mtx);
#endif

    split(in, "\n", env_inputs);
}

std::string svs::svs_query(const std::string& query)
{
    if (state_stack.size() == 0)
    {
        return "";
    }
    return state_stack[0]->get_scene()->parse_query(query);
}

void svs::proxy_get_children(std::map<std::string, cliproxy*>& c)
{
    c["connect_viewer"]    = new memfunc_proxy<svs>(this, &svs::cli_connect_viewer);
    c["connect_viewer"]->set_help("Connect to a running viewer.")
    .add_arg("PORT", "TCP port (or file socket path in Linux) to connect to.")
    ;

    c["disconnect_viewer"] = new memfunc_proxy<svs>(this, &svs::cli_disconnect_viewer);
    c["disconnect_viewer"]->set_help("Disconnect from viewer.");

    c["filters"]           = &get_filter_table();
    c["commands"]          = &get_command_table();

    std::vector<svs_state*>::iterator state_itr = state_stack.begin();
    for (; state_itr != state_stack.end(); state_itr++) {
        c[(*state_itr)->get_name()] = *state_itr;
    }

#ifdef ENABLE_ROS
    c["ros"] = ri;
#endif
#ifdef ENABLE_OPENCV
    c["vibmgr"] = vib_manager;
    c["vltm"] = v_mem_opencv;

    for (size_t j = 0, jend = state_stack.size(); j < jend; ++j)
    {
        c[state_stack[j]->get_name()] = state_stack[j];
    }
#endif
}

bool svs::do_cli_command(const std::vector<std::string>& args, std::string& output)
{
    std::stringstream ss;
    std::vector<std::string> rest;

    if (args.size() < 2)
    {
        output = "specify path\n";
        return false;
    }

    for (size_t i = 2, iend = args.size(); i < iend; ++i)
    {
        rest.push_back(args[i]);
    }

    proxy_use(args[1], rest, ss);
    output = ss.str();
    return true;
}

void svs::cli_connect_viewer(const std::vector<std::string>& args, std::ostream& os)
{
    if (args.empty())
    {
        os << "specify socket path" << std::endl;
        return;
    }
    if (draw->connect(args[0]))
    {
        os << "connection successful" << std::endl;
        for (size_t i = 0, iend = state_stack.size(); i < iend; ++i)
        {
            state_stack[i]->get_scene()->refresh_draw();
        }
    }
    else
    {
        os << "connection failed" << std::endl;
    }
}

void svs::cli_disconnect_viewer(const std::vector<std::string>& args, std::ostream& os)
{
    draw->disconnect();
}
