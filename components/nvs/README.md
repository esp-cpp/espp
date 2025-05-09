# NVS (Non-Volatile Storage) Component

[![Badge](https://components.espressif.com/components/espp/nvs/badge.svg)](https://components.espressif.com/components/espp/nvs)

The `nvs` component provides various utility classes for interfacing with the
ESP `nvs` subsystem to store and retrieve data that persists between power
cycles and power loss.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [NVS Example](#nvs-example)
  - [NVS](#nvs)
  - [NVS Handle](#nvs-handle)
  - [Example](#example)

<!-- markdown-toc end -->

## NVS

The `NVS` component provides a simple class representing an NVS controller. 

## NVS Handle

The `NVSHandle` class manages individual NVS storage handles, allowing for scoped
control over specific NVS namespaces. It simplifies operations like reading,
writing, and committing key-value pairs within these namespaces. 

## Example

The [example](./example) shows the use of the `NVS` component to save a variable to the NVS and load 
it after reset. 

