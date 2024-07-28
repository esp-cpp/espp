import litgen
from srcmlcpp import SrcmlcppOptions
import os


def my_litgen_options() -> litgen.LitgenOptions:
    # configure your options here
    options = litgen.LitgenOptions()

    # ///////////////////////////////////////////////////////////////////
    #  Root namespace
    # ///////////////////////////////////////////////////////////////////
    # The namespace espp is the C++ root namespace for the generated bindings
    # (i.e. no submodule will be generated for it in the python bindings)
    options.namespaces_root = ["espp"]

    # //////////////////////////////////////////////////////////////////
    # Basic functions bindings
    # ////////////////////////////////////////////////////////////////////
    # No specific option is needed for these basic bindings
    # litgen will add the docstrings automatically in the python bindings

    # //////////////////////////////////////////////////////////////////
    # Classes and structs bindings
    # //////////////////////////////////////////////////////////////////
    # No specific option is needed for these bindings.
    # - Litgen will automatically add a default constructor with named parameters
    #   for structs that have no constructor defined in C++.
    #  - A class will publish only its public methods and members

    # ///////////////////////////////////////////////////////////////////
    #  Exclude functions and/or parameters from the bindings
    # ///////////////////////////////////////////////////////////////////
    # We want to exclude `inline void priv_SetOptions(bool v) {}` from the bindings
    # priv_ is a prefix for private functions that we don't want to expose
    options.fn_exclude_by_name__regex = "^priv_"

    # Inside `inline void SetOptions(bool v, bool priv_param = false) {}`,
    # we don't want to expose the private parameter priv_param
    # (it is possible since it has a default value)
    options.fn_params_exclude_names__regex = "^priv_"

    # ////////////////////////////////////////////////////////////////////
    # Override virtual methods in python
    # ////////////////////////////////////////////////////////////////////
    # The virtual methods of this class can be overriden in python
    options.class_template_options.add_specialization(r"Bezier", ["espp::Vector2f"])
    options.class_template_options.add_specialization(r"Bezier::Config", ["espp::Vector2f"])
    options.class_template_options.add_specialization(r"Bezier::WeightedConfig", ["espp::Vector2f"])
    options.class_template_options.add_specialization(r"RangeMapper", ["int", "float"])
    options.class_template_options.add_specialization(r"RangeMapper::Config", ["int", "float"])
    options.class_template_options.add_specialization(r"Vector2d", ["int", "float"])

    # ////////////////////////////////////////////////////////////////////
    # Publish bindings for template functions
    # ////////////////////////////////////////////////////////////////////
    options.fn_template_options.add_specialization(r"^sgn$", ["int", "float"], add_suffix_to_function_name=True)
    options.fn_template_options.add_specialization(r"^to_time_t$", ["std::filesystem::file_time_type"], add_suffix_to_function_name=True)

    # ////////////////////////////////////////////////////////////////////
    # Return values policy
    # ////////////////////////////////////////////////////////////////////
    # `FileSystem& get()` and `EventManager& get()` return references, that
    # python should not free, so we force the reference policy to be 'reference'
    # instead of 'automatic'
    options.fn_return_force_policy_reference_for_references__regex = "^get$"

    # ////////////////////////////////////////////////////////////////////
    #  Boxed types
    # ////////////////////////////////////////////////////////////////////
    # Adaptation for `inline void SwitchBoolValue(bool &v) { v = !v; }`
    # SwitchBoolValue is a C++ function that takes a bool parameter by reference and changes its value
    # Since bool are immutable in python, we can to use a BoxedBool instead
    options.fn_params_replace_modifiable_immutable_by_boxed__regex = "^SwitchBoolValue$"

    # ////////////////////////////////////////////////////////////////////
    #  Published vectorized math functions and namespaces
    # ////////////////////////////////////////////////////////////////////
    # The functions in the MathFunctions namespace will be also published as vectorized functions
    # options.fn_namespace_vectorize__regex = r"^espp::MathFunctions$"  # Do it in this namespace only
    options.fn_vectorize__regex = r".*"  # For all functions

    # ////////////////////////////////////////////////////////////////////
    # Format the python stubs with black
    # ////////////////////////////////////////////////////////////////////
    # Set to True if you want the stub file to be formatted with black
    options.python_run_black_formatter = False

    return options


def autogenerate() -> None:
    repository_dir = os.path.realpath(os.path.dirname(__file__) + "/../")
    output_dir = repository_dir + "/lib/python_bindings"

    include_dir = repository_dir + "/components/"
    header_files = [include_dir + "base_component/include/base_component.hpp",
                    include_dir + "color/include/color.hpp",
                    include_dir + "csv/include/csv.hpp",
                    include_dir + "event_manager/include/event_manager.hpp",
                    include_dir + "file_system/include/file_system.hpp",
                    include_dir + "ftp/include/ftp_server.hpp",
                    include_dir + "math/include/bezier.hpp", # have to set class template options
                    include_dir + "math/include/fast_math.hpp",
                    include_dir + "math/include/gaussian.hpp",
                    include_dir + "math/include/range_mapper.hpp", # have to set class template options
                    include_dir + "math/include/vector2d.hpp", # have to set class template options
                    # include_dir + "logger/include/logger.hpp",
                    include_dir + "rtsp/include/jpeg_frame.hpp",
                    include_dir + "rtsp/include/jpeg_header.hpp",
                    include_dir + "rtsp/include/rtsp_client.hpp",
                    include_dir + "rtsp/include/rtsp_server.hpp",
                    include_dir + "socket/include/socket.hpp",
                    include_dir + "socket/include/tcp_socket.hpp",
                    include_dir + "socket/include/udp_socket.hpp",
                    # include_dir + "state_machine/include/deep_history_state.hpp",
                    # include_dir + "state_machine/include/shallow_history_state.hpp",
                    # include_dir + "state_machine/include/state_base.hpp",
                    include_dir + "task/include/task.hpp",
                    include_dir + "timer/include/timer.hpp",

                    # include_dir + "state_machine/include/magic_enum.hpp",
                    # include_dir + "tabulate/include/tabulate.hpp",
                    # include_dir + "serialization/include/serialization.hpp",
    ]

    litgen.write_generated_code_for_files(
        options=my_litgen_options(),
        input_cpp_header_files=header_files,
        output_cpp_pydef_file=output_dir + "/pybind_espp.cpp",
        output_stub_pyi_file=output_dir + "/espp_lib/__init__.pyi",
    )


if __name__ == "__main__":
    autogenerate()
