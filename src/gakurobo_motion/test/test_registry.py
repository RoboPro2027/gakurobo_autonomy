import pytest

from gakurobo_motion.primitive import Primitive, PrimitiveContext, PrimitiveError
from gakurobo_motion.primitives.delay import Delay
from gakurobo_motion.registry import load_class, load_primitives


class FakeTime:
    def __init__(self, value):
        self.nanoseconds = value


class FakeClock:
    def __init__(self):
        self.value = 0

    def now(self):
        self.value += 10_000_000
        return FakeTime(self.value)


class FakeNode:
    def get_clock(self):
        return getattr(self, 'clock', None) or self._make()

    def _make(self):
        self.clock = FakeClock()
        return self.clock


def context(cancelled=False):
    return PrimitiveContext(lambda: cancelled, lambda phase: None)


def test_yaml_entries_become_primitives(tmp_path):
    path = tmp_path / 'p.yaml'
    path.write_text(
        'primitives:\n  delay: gakurobo_motion.primitives.delay:Delay\n',
        encoding='utf-8',
    )

    primitives = load_primitives([path], FakeNode(), 'br')

    assert isinstance(primitives['delay'], Delay)
    assert primitives['delay'].robot == 'br'


def test_a_class_that_is_not_a_primitive_is_rejected():
    with pytest.raises(TypeError):
        load_class('gakurobo_motion.primitive:PrimitiveError')


def test_delay_finishes_after_its_duration():
    Delay(FakeNode(), 'br').execute({'duration_ms': '30'}, context())


def test_delay_stops_when_cancelled():
    with pytest.raises(PrimitiveError, match='CANCELLED'):
        Delay(FakeNode(), 'br').execute({'duration_ms': '1000'}, context(True))


@pytest.mark.parametrize('parameters', [{}, {'duration_ms': 'x'}, {'duration_ms': '-1'}])
def test_delay_rejects_bad_parameters(parameters):
    with pytest.raises(PrimitiveError):
        Delay(FakeNode(), 'br').execute(parameters, context())


def test_primitive_is_abstract():
    with pytest.raises(TypeError):
        Primitive(FakeNode(), 'br')
