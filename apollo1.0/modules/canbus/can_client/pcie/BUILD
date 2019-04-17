load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pcie_can_client",
    srcs = [
        "pcie_can_client.cc",
    ],
    hdrs = [
        "pcie_can_client.h",
    ],
    deps = [
        "//modules/canbus/can_client",
        "//third_party/can_card_library/pcie_can",
    ],
)

cc_test(
    name = "pcie_can_client_test",
    size = "small",
    srcs = [
        "pcie_can_client_test.cc",
    ],
    deps = [
        "//modules/canbus/can_client/pcie:pcie_can_client",
        "//modules/canbus/common:canbus_common",
        "//modules/common:log",
        "@gtest//:main",
    ],
)

cpplint()
