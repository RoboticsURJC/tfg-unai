#
# Copyright (c) 2022 ZettaScale Technology
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
# which is available at https://www.apache.org/licenses/LICENSE-2.0.
#
# SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
#
# Contributors:
#   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
#

from zenoh_flow.interfaces import Sink
from zenoh_flow import Inputs
from zenoh_flow.types import Context
from typing import Dict, Any


class FileWriter(Sink):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Inputs,
    ):
        self.input = inputs.take("in", str, lambda buf: buf.decode("utf-8"))
        if self.input is None:
            raise ValueError("Unable to find input")
        self.out_file = open("/tmp/greetings.txt", "w+")

    def finalize(self) -> None:
        self.out_file.close()

    async def iteration(self) -> None:
        pass
        #message = await self.input.recv()
        #greeting = message.get_data()
        #if greeting is not None:
        #    self.out_file.write(greeting)
        #    self.out_file.flush()


def register():
    return FileWriter
