import litgen
from srcmlcpp import SrcmlcppOptions
import os
import re

# NOTE: cdr.hpp is intentionally NOT generated here. srcmlcpp cannot parse it, and even when coerced
# the generated CdrReader API is unusable (output-reference reads can't return values, and the
# non-owning std::span would dangle). It is bound by hand instead in
# python_bindings/cdr_bindings.cpp (registered via py_init_cdr in module.cpp).


# ////////////////////////////////////////////////////////////////////////////////////////////////
# Parse-time preprocessing
# ////////////////////////////////////////////////////////////////////////////////////////////////
# srcmlcpp mis-parses a class when its members carry a C++20 trailing `requires` clause (e.g.
# `T angle(...) const requires std::is_floating_point<T>::value { ... }` in Vector2d). The class
# scope is lost, so litgen emits members as stray free functions and does not specialize the
# template class at all. The constraint is irrelevant to bindings, so we strip trailing requires
# clauses before parsing. The pattern is anchored to a function signature `) [const] [noexcept]`
# (and stops at the body `{` or `;`) so it never touches the word "requires" in comments or strings.
_TRAILING_REQUIRES = re.compile(r"(\)\s*(?:const\s*)?(?:noexcept\s*)?)requires\b[^{};]*?(?=[{;])")


def _code_preprocess(code: str) -> str:
    return _TRAILING_REQUIRES.sub(r"\1", code)


# ////////////////////////////////////////////////////////////////////////////////////////////////
# Post-processing of the generated pybind code
# ////////////////////////////////////////////////////////////////////////////////////////////////
# litgen/srcmlcpp have two bugs we cannot fix via options (see lib/README):
#   * srcML inner-struct bug (srcML #2033): classes with a nested Config/etc. struct get a bogus
#     `py::init<>() // implicit default constructor`, even though they are not default-constructible.
#   * Nested structs of *template* classes (RangeMapper<T>::Config, Bezier<T>::Config) and the
#     pyClass variable references for them are emitted without the template argument.
# These used to be fixed by hand after every regeneration. _postprocess_generated() reapplies them
# automatically so the generated file compiles as-is.

# Map of "main class" pyClass chain-head variable -> replacement for its bogus implicit default
# constructor. An empty string removes the constructor entirely (class is not constructible here).
_IMPLICIT_CTOR_FIX = {
    "pyClassLogger": ".def(py::init<const espp::Logger::Config &>())",
    "pyClassGaussian": ".def(py::init<const espp::Gaussian::Config &>())",
    "pyClassPid": ".def(py::init<const espp::Pid::Config &>())",
    "pyClassTcpSocket": ".def(py::init<const espp::TcpSocket::Config &>())",
    "pyClassUdpSocket": ".def(py::init<const espp::UdpSocket::Config &>())",
    "pyClassJoystick": ".def(py::init<const espp::Joystick::Config &>())",
    "pyClassTimer": (
        ".def(py::init<const espp::Timer::Config &>())\n"
        "      .def(py::init<const espp::Timer::AdvancedConfig &>())"
    ),
    "pyClassBezier_espp_Vector2f": (
        ".def(py::init<const espp::Bezier<espp::Vector2f>::Config &>())\n"
        "      .def(py::init<const espp::Bezier<espp::Vector2f>::WeightedConfig &>())"
    ),
    "pyClassTask": (
        ".def(py::init<const espp::Task::callback_no_params_fn &, "
        "const ::espp::Task::BaseConfig &>())\n"
        "      .def(py::init<const espp::Task::callback_no_params_fn &, "
        "const ::espp::Task::BaseConfig &, espp::Logger::Verbosity>())"
    ),
    "pyClassSocket": "",  # base class: drop the bogus default constructor entirely
    # RTSP: abstract base packetizers must not be constructible; the concrete ones take a Config.
    "pyClassRtpDepacketizer": "",  # abstract base
    "pyClassRtpPacketizer": "",  # abstract base
    "pyClassGenericDepacketizer": ".def(py::init<const espp::GenericDepacketizer::Config &>())",
    "pyClassGenericPacketizer": ".def(py::init<const espp::GenericPacketizer::Config &>())",
    "pyClassH264Depacketizer": ".def(py::init<const espp::H264Depacketizer::Config &>())",
    "pyClassH264Packetizer": ".def(py::init<const espp::H264Packetizer::Config &>())",
    "pyClassMjpegDepacketizer": ".def(py::init<const espp::MjpegDepacketizer::Config &>())",
    "pyClassMjpegPacketizer": ".def(py::init<const espp::MjpegPacketizer::Config &>())",
    "pyClassRtspClient": ".def(py::init<const espp::RtspClient::Config &>())",
    "pyClassRtspServer": ".def(py::init<const espp::RtspServer::Config &>())",
    "pyClassRtspSession": (
        ".def(py::init<std::shared_ptr<espp::TcpSocket>, const espp::RtspSession::Config &>())"
    ),
}


def _fix_implicit_default_ctors(code: str) -> str:
    for chain_head, replacement in _IMPLICIT_CTOR_FIX.items():
        pattern = re.compile(
            r"\n  " + re.escape(chain_head) + r"\n"
            r"      \.def\(py::init<>\(\)\) // implicit default constructor\n"
        )
        new_line = (f"\n  {chain_head}\n      {replacement}\n") if replacement else f"\n  {chain_head}\n"
        code, n = pattern.subn(lambda m, nl=new_line: nl, code)
        if n != 1:
            print(f"WARNING: implicit-ctor fix for {chain_head} applied {n} times (expected 1)")
    return code


def _fix_template_class_nested(code: str) -> str:
    # Bezier: single specialization (espp::Vector2f). The outer class is already emitted correctly,
    # but its nested Config/WeightedConfig and the bare `pyClassBezier` references are not. These
    # patterns occur only in the Bezier section, so a global replace is safe. `(?!\w)` avoids
    # touching the correctly-suffixed variables (pyClassBezier_espp_Vector2f, *_ClassConfig).
    code = code.replace("espp::Bezier::", "espp::Bezier<espp::Vector2f>::")
    code = re.sub(r"pyClassBezier(?!\w)", "pyClassBezier_espp_Vector2f", code)

    # RangeMapper: two specializations (<int>, <float>) share the raw `espp::RangeMapper::` text and
    # bare `pyClassRangeMapper`, so they must be fixed per-block.
    code = _fix_specialized_block(
        code, "pyClassRangeMapper_int", "pyClassRangeMapper_float",
        "espp::RangeMapper", "int", "pyClassRangeMapper",
    )
    code = _fix_specialized_block(
        code, "pyClassRangeMapper_float", None,
        "espp::RangeMapper", "float", "pyClassRangeMapper",
    )

    # Vector2d: two specializations (<int>, <float>), same nested/bare-reference issue as RangeMapper.
    code = _fix_specialized_block(
        code, "pyClassVector2d_int", "pyClassVector2d_float",
        "espp::Vector2d", "int", "pyClassVector2d",
    )
    code = _fix_specialized_block(
        code, "pyClassVector2d_float", None,
        "espp::Vector2d", "float", "pyClassVector2d",
    )
    return code


def _fix_specialized_block(code, start_var, next_var, raw_type, targ, bare_pyclass):
    start = code.find(f"auto {start_var} =")
    if start < 0:
        return code
    if next_var is not None:
        end = code.find(f"auto {next_var} =", start)
    else:
        # end of this specialization's block: the next top-level `  auto pyClass...` declaration.
        m = re.search(r"\n  auto pyClass", code[start + 1:])
        end = (start + 1 + m.start()) if m else len(code)
    if end < 0:
        end = len(code)
    block = code[start:end]
    # Add the template argument to every bare use of the class (both `raw_type::Member` and
    # bare-type uses like `const raw_type &` / `py::class_<raw_type>`). `(?!<)` skips uses that are
    # already specialized (e.g. the `py::class_<espp::Vector2d<int>>` header itself).
    block = re.sub(re.escape(raw_type) + r"(?!<)", f"{raw_type}<{targ}>", block)
    block = re.sub(re.escape(bare_pyclass) + r"(?!\w)", f"{start_var}", block)
    return code[:start] + block + code[end:]


# Methods that exist as BOTH an instance and a static overload of the same name (e.g.
# `Task::get_id()` and `static Task::get_id(const Task&)`). litgen binds both, but pybind11 refuses
# to register a static and instance method under one name (fails at import). Drop the static one.
_STATIC_INSTANCE_DUP_METHODS = ["get_id"]


def _remove_static_instance_dups(code: str) -> str:
    for name in _STATIC_INSTANCE_DUP_METHODS:
        # Remove the `.def_static("name", ... )` block up to (not including) the next binding.
        code = re.sub(
            r"\n      \.def_static\(\"" + re.escape(name) + r"\",.*?(?=\n      \.def)",
            "",
            code,
            flags=re.DOTALL,
        )
    return code


# litgen drops the base class and std::shared_ptr holder for the RTP packetizer hierarchy and
# JpegFrame. These are returned/stored as std::shared_ptr (callbacks, RtpDepacketizer factories), so
# pybind needs the shared_ptr holder, and the inheritance so derived/base convert. Map class name ->
# full `py::class_<...>` template arguments (base classes must precede the holder, and a base must be
# registered before its derived class, which matches the header/generation order).
_CLASS_HOLDER_FIX = {
    "JpegFrame": "espp::JpegFrame, std::shared_ptr<espp::JpegFrame>",
    "RtpDepacketizer": "espp::RtpDepacketizer, std::shared_ptr<espp::RtpDepacketizer>",
    "RtpPacketizer": "espp::RtpPacketizer, std::shared_ptr<espp::RtpPacketizer>",
    "GenericDepacketizer":
        "espp::GenericDepacketizer, espp::RtpDepacketizer, std::shared_ptr<espp::GenericDepacketizer>",
    "GenericPacketizer":
        "espp::GenericPacketizer, espp::RtpPacketizer, std::shared_ptr<espp::GenericPacketizer>",
    "H264Depacketizer":
        "espp::H264Depacketizer, espp::RtpDepacketizer, std::shared_ptr<espp::H264Depacketizer>",
    "H264Packetizer":
        "espp::H264Packetizer, espp::RtpPacketizer, std::shared_ptr<espp::H264Packetizer>",
    "MjpegDepacketizer":
        "espp::MjpegDepacketizer, espp::RtpDepacketizer, std::shared_ptr<espp::MjpegDepacketizer>",
    "MjpegPacketizer":
        "espp::MjpegPacketizer, espp::RtpPacketizer, std::shared_ptr<espp::MjpegPacketizer>",
}


def _fix_class_holders(code: str) -> str:
    for name, args in _CLASS_HOLDER_FIX.items():
        # The trailing '>' ensures we match the bare `py::class_<espp::Name>` and not
        # `py::class_<espp::Name::Config>`.
        bare = f"py::class_<espp::{name}>"
        replacement = f"py::class_<{args}>"
        if bare in code:
            code = code.replace(bare, replacement)
        else:
            print(f"WARNING: class-holder fix for {name} did not match {bare}")
    return code


def _postprocess_generated(code: str) -> str:
    code = _fix_implicit_default_ctors(code)
    code = _fix_template_class_nested(code)
    code = _remove_static_instance_dups(code)
    code = _fix_class_holders(code)
    return code


def my_litgen_options() -> litgen.LitgenOptions:
    # configure your options here
    options = litgen.LitgenOptions()

    # ///////////////////////////////////////////////////////////////////
    #  Root namespace
    # ///////////////////////////////////////////////////////////////////
    # The namespace espp is the C++ root namespace for the generated bindings
    # (i.e. no submodule will be generated for it in the python bindings)
    options.namespaces_root = ["espp"]
    # we don't actualy want to exclude the detail namespace
    options.namespace_exclude__regex = r"[Ii]nternal" # default was r"[Ii]nternal|[Dd]etail"

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
    # To prevent the generation of the default constructor with named parameters
    # for a specific struct, you can use the following option:
    options.struct_create_default_named_ctor__regex = r".*" # default
    options.class_create_default_named_ctor__regex = r".*"

    # ///////////////////////////////////////////////////////////////////
    #  Exclude functions and/or parameters from the bindings
    # ///////////////////////////////////////////////////////////////////
    # We want to exclude `inline void priv_SetOptions(bool v) {}` from the bindings
    # priv_ is a prefix for private functions that we don't want to expose
    # options.fn_exclude_by_name__regex = "run_on_core" # NOTE: this doesn't work since it seems to be parsing that fails for Task::run_on_core
    # Exclude Vector2d's floating-point-only members. They are guarded by a `requires` clause that
    # we strip at parse time (see _code_preprocess) so the class can be parsed; without the clause
    # litgen would otherwise bind them for Vector2d<int> too, which does not compile. These names are
    # unique to Vector2d, so excluding them globally is safe.
    options.fn_exclude_by_name__regex = r"^(inv_magnitude|angle|signed_angle|rotated)$"

    # we'd like the following classes to be able to pick up new attributes
    # dynamically (within python):
    # -
    options.class_dynamic_attributes__regex = r".*" # expose all classes to support dynamic attributes

    # Inside `inline void SetOptions(bool v, bool priv_param = false) {}`,
    # we don't want to expose the private parameter priv_param
    # (it is possible since it has a default value)
    # options.fn_params_exclude_names__regex = "^priv_"

    # ////////////////////////////////////////////////////////////////////
    # Override virtual methods in python
    # ////////////////////////////////////////////////////////////////////
    # The virtual methods of this class can be overriden in python
    options.class_template_options.add_specialization(r"WeightedConfig", ["espp::Vector2f"]) # NOTE: this doesn't seem to work
    options.class_template_options.add_specialization(r"Bezier", ["espp::Vector2f"])
    options.class_template_options.add_specialization(r"Bezier::Config", ["espp::Vector2f"]) # NOTE: this doesn't seem to work
    options.class_template_options.add_specialization(r"RangeMapper", ["int", "float"])
    options.class_template_options.add_specialization(r"RangeMapper::Config", ["int", "float"]) # NOTE: this doesn't seem to work
    options.class_template_options.add_specialization(r"Vector2d", ["int", "float"]) # NOTE: this still generates some bindings which are not specialized for some reason

    # ////////////////////////////////////////////////////////////////////
    # Publish bindings for template functions
    # ////////////////////////////////////////////////////////////////////
    options.fn_template_options.add_specialization(r"^sgn$", ["int", "float"], add_suffix_to_function_name=False)

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

    # ////////////////////////////////////////////////////////////////////
    # Parse-time preprocessing (strip trailing C++20 requires-clauses)
    # ////////////////////////////////////////////////////////////////////
    # Without this, srcmlcpp mis-parses Vector2d (its members use trailing requires-clauses) and the
    # `Vector2d` template specializations above silently fail to generate. See _code_preprocess.
    options.srcmlcpp_options.code_preprocess_function = _code_preprocess

    return options


def autogenerate() -> None:
    repository_dir = os.path.realpath(os.path.dirname(__file__) + "/../")
    output_dir = repository_dir + "/lib/python_bindings"

    include_dir = repository_dir + "/components/"
    header_files = [include_dir + "base_component/include/base_component.hpp",
                    # cdr.hpp is bound by hand in python_bindings/cdr_bindings.cpp (see note above).
                    include_dir + "cobs/include/cobs.hpp",
                    include_dir + "cobs/include/cobs_stream.hpp",
                    include_dir + "color/include/color.hpp",
                    include_dir + "event_manager/include/event_manager.hpp",
                    # include_dir + "file_system/include/file_system.hpp", # can't deal with singleton that does not support constructor / destructor
                    include_dir + "ftp/include/ftp_server.hpp",
                    # include_dir + "ftp/include/ftp_client_session.hpp", can't deal with tcpsocket unique ptr in constructor
                    include_dir + "logger/include/logger.hpp",
                    include_dir + "math/include/bezier.hpp", # have to set class template options
                    include_dir + "math/include/fast_math.hpp",
                    include_dir + "math/include/gaussian.hpp",
                    include_dir + "math/include/range_mapper.hpp", # have to set class template options
                    include_dir + "math/include/vector2d.hpp", # have to set class template options
                    include_dir + "ndef/include/ndef.hpp",
                    include_dir + "pid/include/pid.hpp",
                    include_dir + "socket/include/socket.hpp",
                    include_dir + "socket/include/tcp_socket.hpp",
                    include_dir + "socket/include/udp_socket.hpp",
                    include_dir + "task/include/task.hpp",
                    include_dir + "timer/include/timer.hpp",

                    # NOTE: this must come after vector2d.hpp and range_mapper.hpp since it depends on them!
                    include_dir + "joystick/include/joystick.hpp",

                    # NOTE: RtpsParticipant has many nested types (GuidPrefix, EntityId,
                    # Locator::Kind, ...), std::function callbacks, and std::span APIs that litgen
                    # does not bind cleanly (it emits unqualified nested type names that do not
                    # compile). Like cdr, it is best exposed via a hand-written shim; deferred for
                    # now so the rest of the module generates and compiles.
                    # include_dir + "rtps/include/rtps.hpp",

                    # NOTE: this must come after socket since it depends on it!
                    include_dir + "rtsp/include/rtp_types.hpp",
                    include_dir + "rtsp/include/rtp_depacketizer.hpp",
                    include_dir + "rtsp/include/rtp_packetizer.hpp",
                    include_dir + "rtsp/include/generic_depacketizer.hpp",
                    include_dir + "rtsp/include/generic_packetizer.hpp",
                    include_dir + "rtsp/include/h264_depacketizer.hpp",
                    include_dir + "rtsp/include/h264_packetizer.hpp",
                    include_dir + "rtsp/include/mjpeg_depacketizer.hpp",
                    include_dir + "rtsp/include/mjpeg_packetizer.hpp",
                    include_dir + "rtsp/include/rtp_jpeg_packet.hpp",
                    include_dir + "rtsp/include/jpeg_frame.hpp",
                    include_dir + "rtsp/include/jpeg_header.hpp",
                    include_dir + "rtsp/include/rtcp_packet.hpp",
                    include_dir + "rtsp/include/rtp_packet.hpp",
                    include_dir + "rtsp/include/rtsp_client.hpp",
                    include_dir + "rtsp/include/rtsp_server.hpp",
                    include_dir + "rtsp/include/rtsp_session.hpp",

                    # filters:
                    include_dir + "filters/include/lowpass_filter.hpp",
                    include_dir + "filters/include/simple_lowpass_filter.hpp",

                    # these are template classes so I've not ipmlemented them into python bindings yet
                    # include_dir + "filters/include/transfer_function.hpp",
                    # include_dir + "filters/include/sos_filter.hpp",
                    # include_dir + "filters/include/biquad_filter.hpp",
                    # include_dir + "filters/include/butterworth_filter.hpp",

                    # state machine:
                    # include_dir + "state_machine/include/deep_history_state.hpp",
                    # include_dir + "state_machine/include/shallow_history_state.hpp",
                    # include_dir + "state_machine/include/state_base.hpp",
                    # include_dir + "state_machine/include/magic_enum.hpp",

                    # csv (template header):
                    # include_dir + "csv/include/csv.hpp",

                    # tabulate (template header):
                    # include_dir + "tabulate/include/tabulate.hpp",

                    # serialization (template header):
                    # include_dir + "serialization/include/serialization.hpp",
    ]

    pydef_file = output_dir + "/pybind_espp.cpp"
    litgen.write_generated_code_for_files(
        options=my_litgen_options(),
        input_cpp_header_files=header_files,
        output_cpp_pydef_file=pydef_file,
        output_stub_pyi_file=output_dir + "/espp/__init__.pyi",
    )

    # Reapply the fixes that litgen/srcmlcpp cannot do via options, so the generated file compiles
    # without any manual editing (see _postprocess_generated).
    with open(pydef_file, "r") as f:
        code = f.read()
    code = _postprocess_generated(code)
    with open(pydef_file, "w") as f:
        f.write(code)
    print(f"Post-processed {pydef_file}")

    # Apply the compiler-driven nested-scope qualification fixes (RTSP/rtps), so the generated file
    # compiles with zero manual edits. Requires a configured build (compile_commands.json); if that
    # is missing, skip with a hint rather than failing.
    try:
        import fix_generated_bindings
        if os.path.exists(fix_generated_bindings.COMPILE_DB):
            print("Applying compiler-driven qualification fixes...")
            fix_generated_bindings.main()
        else:
            print(
                "Skipping qualification fixer: no build/compile_commands.json. Configure once with\n"
                "  cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON .. (from lib/build), then run\n"
                "  python fix_generated_bindings.py"
            )
    except Exception as exc:  # noqa: BLE001 - generation should not hard-fail on the optional fixer
        print(f"Qualification fixer did not run ({exc}); run python fix_generated_bindings.py manually.")


if __name__ == "__main__":
    autogenerate()
