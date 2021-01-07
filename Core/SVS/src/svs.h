#ifndef SVS_H
#define SVS_H

#include <vector>
#include <map>
#include <set>
#include "soar_interface.h"
#include "sgnode.h"
#include "common.h"
#include "svs_interface.h"
#include "cliproxy.h"
#include "imagination.h"
#ifdef ENABLE_OPENCV
#include <opencv2/opencv.hpp>
#include "vision_interface.h"
#endif
#ifdef ENABLE_ROS
#include <boost/thread.hpp>
#include "ros_interface.h"
#endif

// FORWARD DECLARATIONS
class command;
class scene;
class drawer;
class image_descriptor;

#ifdef ENABLE_ROS
class pcl_image;
#endif
#ifdef ENABLE_OPENCV
class opencv_image;
#endif
class basic_image;

template<typename img_t>
class exact_visual_archetype;

template <typename img_T, template<typename T> class atype_T>
class visual_memory;

class svs;


//////////////////
// END PREAMBLE //
//////////////////

/* working memory scene graph object - mediates between wmes and scene graph nodes */
class sgwme : public sgnode_listener
{
    public:
        sgwme(soar_interface* si, Symbol* ident, sgwme* parent, sgnode* node);
        ~sgwme();
        void node_update(sgnode* n, sgnode::change_type t, const std::string& update_info);
        Symbol* get_id()
        {
            return id;
        }
        sgnode* get_node()
        {
            return node;
        }
        std::map<sgwme*, wme*>* get_childs()
        {
            return &childs;
        }

    private:
        void add_child(sgnode* c);
        
        // Functions dealing with maintaining tags on sgnodes
        void update_tag(const std::string& tag_name);
        void delete_tag(const std::string& tag_name);
        void set_tag(const std::string& tag_name, const std::string& tag_value);
        
        sgwme*          parent;
        sgnode*         node;
        Symbol*         id;
        wme*            id_wme;
        soar_interface* soarint;
        
        std::map<sgwme*, wme*> childs;
        
        std::map<std::string, wme*> tags;
};

struct command_entry
{
    std::string id;
    command* cmd;
    wme* cmd_wme;
    command_entry(std::string id, command* cmd, wme* cmd_wme): id(id), cmd(cmd), cmd_wme(cmd_wme) {}
    bool operator < (const command_entry e) const
    {
        return id.compare(e.id) < 0;
    }
};
typedef std::set<command_entry> command_set;
typedef command_set::iterator command_set_it;


/////////////////////
// SVS_STATE CLASS //
/////////////////////


/**
 * @brief Represents a state in the SVS state stack, Like working memory, SVS
 * can have multiple "states" held in a state stack. In general, there is a
 * separate SVS state for each state in orking memory. Each state in the state
 * stack has its own SVS link, scene, visual input, etc. These states handle
 * are what receive filters, visual operations, and so on. 
 * 
 */
class svs_state : public cliproxy
{
    public:
        /**
         * @brief Construct a new svs state object as the root of the svs state
         * stack.
         * 
         * @param svsp A pointer to the `svs` object whose state stack this 
         * state will be the root of. Essentially the `svs` object which owns
         * this state and its children.
         * @param state The working memory `Symbol` representing this state in
         * working memory. 
         * @param soar An interface for interacting with Soar's working memory.
         * @param scn A pointer to a scene graph.
         */
        svs_state(svs* svsp, Symbol* state, soar_interface* soar, scene* scn);

        /**
         * @brief Construct a new svs state object which is not the root of its
         * state stack.
         * 
         * @param state The working memory `Symbol` representing this state in
         * working memory. 
         * @param parent The `svs_state` in the state stack which is the parent 
         * of this one.
         */
        svs_state(Symbol* state, svs_state* parent);
        
        ~svs_state();
        
        void           process_cmds();
        void           update_cmd_results(int command_type);
        void           update_scene_num();
        void           clear_scene();
        
        std::string    get_name() const
        {
            return name;
        }
        int            get_level() const
        {
            return level;
        }
        int            get_scene_num() const
        {
            return scene_num;
        }
        scene*         get_scene() const
        {
            return scn;
        }

#ifdef ENABLE_ROS
        pcl_image*         get_image_pcl() const
        {
            return img_pcl;
        }
#endif

#ifdef ENABLE_OPENCV
        opencv_image* get_image_opencv() const
        {
            return img_opencv;
        }
        imagination_opencv* get_imagination()
        {
            return &imagination;
        }
#endif

        basic_image*       get_image_basic() const
        {
            return img;
        }
        Symbol*        get_state()
        {
            return state;
        }
        svs*           get_svs()
        {
            return svsp;
        }
        
        /*
         Should only be called by svs::state_deletion_callback to save top-state scene
         during init.
        */
        void disown_scene();
        
    private:
        void init();
        void collect_cmds(Symbol* id, std::set<wme*>& all_cmds);
        
        void proxy_get_children(std::map<std::string, cliproxy*>& c);
        void cli_out(const std::vector<std::string>& args, std::ostream& os);

        std::string     name;
        svs*            svsp;
        int             level;
        svs_state*      parent;
        scene*          scn;
        sgwme*          root;
        image_descriptor* imwme;
        soar_interface* si;
#ifdef ENABLE_ROS
        pcl_image*      img_pcl;
#endif
#ifdef ENABLE_OPENCV
        opencv_image*   img_opencv;
        imagination_opencv imagination;
#endif
        basic_image*    img;

        Symbol* state;
        Symbol* svs_link;
        Symbol* scene_link;
        Symbol* img_link;
        Symbol* cmd_link;
        Symbol* imagine_link;
        
        int scene_num;
        wme* scene_num_wme;
        
        /* command changes per decision cycle */
        command_set curr_cmds;
};


///////////////
// SVS CLASS //
///////////////

class svs : public svs_interface, public cliproxy
{
    public:
        svs(agent* a);
        ~svs();
        
        void state_creation_callback(Symbol* goal);
        void state_deletion_callback(Symbol* goal);
        void output_callback();
        void input_callback();
        void add_input(const std::string& in);
        std::string svs_query(const std::string& query);

        /**
         * @brief Gets the root `svs_state` from the `state_stack`.
         * 
         * @note The "root" of the state stack is actually the *most recent*
         * `svs_state` object to be created, the top of the stack.
         */
        svs_state* get_root_state() { return state_stack.at(0); }
        svs_state* get_last_state() { return state_stack.at(state_stack.size() - 1); }

        typedef visual_memory<basic_image, exact_visual_archetype> exact_basic_mem;
        exact_basic_mem* get_v_mem_basic() { return v_mem_basic; }

#ifdef ENABLE_OPENCV
        typedef visual_memory<opencv_image, exact_visual_archetype> exact_opencv_mem;

        void image_callback(const cv::Mat& new_img);
        exact_opencv_mem* get_v_mem_opencv() { return v_mem_opencv; }
#endif
#ifdef ENABLE_ROS
        typedef visual_memory<pcl_image, exact_visual_archetype> exact_pcl_mem;

        void image_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& new_img);
        exact_pcl_mem* get_v_mem_pcl() {  return v_mem_pcl; }
#endif

        soar_interface* get_soar_interface()
        {
            return si;
        }

        vision_interface* get_vision_interface() {
            return vi;
        }

        drawer* get_drawer() const
        {
            return draw;
        }
        
        bool do_cli_command(const std::vector<std::string>& args, std::string& output);
        
        bool is_enabled()
        {
            return enabled;
        }
        void set_enabled(bool is_enabled)
        {
            enabled = is_enabled;
        }
        
        std::string get_output() const
        {
            return "";
        }

        // dirty bit is true only if there has been a new command
        //   from soar or from SendSVSInput 
        //   (no need to recheck filters)
        static void mark_filter_dirty_bit()
        {
            svs::filter_dirty_bit = true;
        }

        static bool get_filter_dirty_bit()
        {
            return svs::filter_dirty_bit;
        }
        
    private:
        void proc_input(svs_state* s);
        
        void proxy_get_children(std::map<std::string, cliproxy*>& c);
        void cli_connect_viewer(const std::vector<std::string>& args, std::ostream& os);
        void cli_disconnect_viewer(const std::vector<std::string>& args, std::ostream& os);

#ifdef ENABLE_ROS
        ros_interface*            ri;
        boost::mutex              input_mtx;
       
        exact_pcl_mem*            v_mem_pcl;
#endif

#ifdef ENABLE_OPENCV
        vision_interface*         vi;

        exact_opencv_mem*         v_mem_opencv;
#endif

        exact_basic_mem*          v_mem_basic;

        soar_interface*           si;
        std::vector<svs_state*>   state_stack;
        std::vector<std::string>  env_inputs;
        std::string               env_output;
        mutable drawer*           draw;
        scene*                    scn_cache;      // temporarily holds top-state scene during init
        
        bool enabled;

        static bool filter_dirty_bit;
};

#endif
