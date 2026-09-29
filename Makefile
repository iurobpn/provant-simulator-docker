img?=provant

.PHONY: all provant

all: provant

provant:
	docker build -t $(img) .

