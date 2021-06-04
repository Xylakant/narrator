#![cfg_attr(not(feature = "use-std"), no_std)]

use serde::Serialize;
use serde::ser::{Serializer, SerializeTuple};
pub use managed::Managed;

#[cfg(feature = "use-std")]
use serde::{
    Deserialize,
    de::{Deserializer, Visitor, SeqAccess},
};

#[derive(Serialize, Debug)]
#[cfg_attr(feature = "use-std", derive(Deserialize))]
pub struct DataReport<'a> {
    pub timestamp: u32,

    #[serde(serialize_with = "slicer")]
    #[cfg_attr(feature = "use-std", serde(deserialize_with = "unslicer"))]
    pub payload: Managed<'a, [u8; 4096]>,
}

fn slicer<'a, S>(pb: &'a Managed<'a, [u8; 4096]>, s: S) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    let mut seq = s.serialize_tuple(4096)?;
    for element in pb.iter() {
        seq.serialize_element(element)?;
    }
    seq.end()
}

#[cfg(feature = "use-std")]
pub struct BVisitor;

#[cfg(feature = "use-std")]
impl<'de> Visitor<'de> for BVisitor {
    type Value = Managed<'static, [u8; 4096]>;

    fn expecting(&self, _: &mut core::fmt::Formatter<'_>) -> Result<(), std::fmt::Error> {
        Ok(())
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        let mut data = Box::new([0u8; 4096]);
        data.iter_mut().try_for_each(|b| {
            *b = seq.next_element()?.unwrap();
            Ok(())
        })?;
        Ok(Managed::Owned(data))
    }
}

#[cfg(feature = "use-std")]
fn unslicer<'de, D>(des: D) -> Result<Managed<'static, [u8; 4096]>, D::Error>
where
    D: Deserializer<'de>,
{
    des.deserialize_tuple(4096, BVisitor)
}

#[cfg(test)]
mod test {
    use crate::DataReport;
    use managed::Managed;
    use postcard::{to_stdvec, from_bytes};
    use core::ops::Deref;

    #[test]
    fn basic_roundtrip() {
        let foo = DataReport {
            timestamp: 0x12345678,
            payload: Managed::Owned(Box::new([0x42; 4096])),
        };

        let bar = to_stdvec(&foo).unwrap();

        let baz: DataReport = from_bytes(&bar).unwrap();
        assert_eq!(foo.timestamp, baz.timestamp);
        assert_eq!(foo.payload.deref(), baz.payload.deref());
    }
}
