https://pypi.org/project/pypotrace/
In order to install pypotrace, we need to install some ubuntu libraries first, as pypotrace needs bindings:

$ sudo apt-get install build-essential libagg-dev libpotrace-dev pkg-config
(This:)
$ sudo apt-get install python-dev

Then we can install pypotrace (numpy has to be installed)
$ git clone https://github.com/flupke/pypotrace.git
$ cd pypotrace
$ pip install .